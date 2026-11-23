#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import struct
import json
import time
import argparse
from pathlib import Path
from datetime import datetime
import csv
from collections import deque
import numpy as np

# ==== BNO055 / I2C ====
import board
import adafruit_bno055

# ==========================================================
# CONFIG
# ==========================================================
SAMPLE_RATE = 80.0              # Hz
LOG_BASENAME = "sensor_log"     # CSV prefix
OFFSETS_FILE = "bno055_offsets.json"

# Stabilization phase (to define yaw reference)
STABILIZATION_SAMPLES  = 50
STABILITY_THRESHOLD    = 1.1    # max circular std-dev (deg)
CONFIRMATION_WAIT_TIME = 2.0    # seconds stable before accepting

# BNO055 mode
USE_NDOF_MODE = True   # True=NDOF (with magnetometer), False=IMUPLUS (no mag)

# Additional CSV fields, in the native BNO055 axis convention.
SENSOR_VECTORS = (
    ("quaternion", ("quat_w", "quat_x", "quat_y", "quat_z")),
    ("acceleration", ("acc_x_mps2", "acc_y_mps2", "acc_z_mps2")),
    ("gyro", ("gyro_x_radps", "gyro_y_radps", "gyro_z_radps")),
    ("magnetic", ("mag_x_uT", "mag_y_uT", "mag_z_uT")),
    ("linear_acceleration", ("lin_acc_x_mps2", "lin_acc_y_mps2", "lin_acc_z_mps2")),
    ("gravity", ("gravity_x_mps2", "gravity_y_mps2", "gravity_z_mps2")),
)

# File flushing
FLUSH_EVERY_SAMPLES = 80  # ~1 second at 80 Hz

# Status JSON refresh limit (seconds)
STATUS_MIN_PERIOD_S = 0.2


# ==========================================================
# STATUS JSON (atomic)
# ==========================================================
def atomic_write_json(path: Path, obj: dict):
    try:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(path.suffix + ".tmp")
        with open(tmp, "w") as f:
            json.dump(obj, f, ensure_ascii=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except Exception:
        # best-effort; never crash logger for status failures
        pass
# ==========================================================
# HELPERS
# ==========================================================
def apply_offsets(sensor, offsets):
    try:
        sensor.offsets_magnetometer = offsets["magnetometer"]
        sensor.offsets_gyroscope = offsets["gyroscope"]
        sensor.offsets_accelerometer = offsets["accelerometer"]
    except Exception as e:
        raise SystemExit(f"Could not apply calibration offsets: {e}") from e

    print("[INFO] Offsets applied.")
    print(f"       MAG={offsets['magnetometer']}")
    print(f"       GYR={offsets['gyroscope']}")
    print(f"       ACC={offsets['accelerometer']}")


def load_offsets_from_json(path: Path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, dict):
            raise ValueError("Expected a JSON object.")
        offsets = {}
        for name in ("magnetometer", "gyroscope", "accelerometer"):
            values = data.get(name)
            if (not isinstance(values, list) or len(values) != 3
                    or any(type(value) is not int or not -32768 <= value <= 32767
                           for value in values)):
                raise ValueError(f"{name} must contain three signed 16-bit integers.")
            offsets[name] = tuple(values)
        return offsets
    except (OSError, ValueError) as e:
        raise SystemExit(
            f"Calibration file missing or invalid: {path} ({e}). "
            "Run calibrate_BNO055.py before starting the logger."
        ) from e

def read_heading_deg(bno):
    e = bno.euler
    return float(e[0]) if e and e[0] is not None else None

def read_sample(device, buffer, magnetic_enabled):
    """Read page 0 registers 0x08 through 0x35 in one I2C transaction."""
    with device as bus:
        bus.write_then_readinto(b"\x08", buffer)

    heading, roll, pitch = struct.unpack_from("<hhh", buffer, 0x1A - 0x08)
    calibration = buffer[0x35 - 0x08]
    levels = tuple((calibration >> shift) & 3 for shift in (6, 4, 2, 0))
    temperature = struct.unpack_from("<b", buffer, 0x34 - 0x08)[0]

    # Match the Adafruit driver's default units and the existing CSV columns.
    register_scales = (
        (0x20, 1 / 16384),
        (0x08, 1 / 100),
        (0x14, 0.001090830782496456),
        (0x0E, 1 / 16),
        (0x28, 1 / 100),
        (0x2E, 1 / 100),
    )
    readings = {}
    for (name, columns), (register, scale) in zip(SENSOR_VECTORS, register_scales):
        if name == "magnetic" and not magnetic_enabled:
            continue
        values = struct.unpack_from("<" + "h" * len(columns), buffer, register - 0x08)
        readings.update(zip(columns, (value * scale for value in values)))

    return (heading / 16, roll / 16, -pitch / 16), levels, float(temperature), readings


def angle_wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0

def angle_diff_deg(a, b):
    return angle_wrap_deg(a - b)

def circular_mean_std_deg(values_deg):
    vals = np.deg2rad(np.array(values_deg, dtype=float))
    C = np.cos(vals).mean()
    S = np.sin(vals).mean()
    mean_rad = np.arctan2(S, C)
    R = np.hypot(C, S)
    std_rad = np.sqrt(-2.0 * np.log(R)) if R > 0 else np.pi
    return np.rad2deg(mean_rad), np.rad2deg(std_rad)

def stabilize_and_get_yaw_reference(bno, status_path: Path, status_state: dict):
    print("[INFO] Stabilizing... keep the sensor still.")
    recent_yaws = deque(maxlen=STABILIZATION_SAMPLES)
    consecutive_stable = 0
    required_stable = int(CONFIRMATION_WAIT_TIME * SAMPLE_RATE)
    collecting_msg = False

    last_status_t = 0.0

    while True:
        yaw = read_heading_deg(bno)
        if yaw is None:
            time.sleep(1.0 / SAMPLE_RATE)
            continue

        recent_yaws.append(yaw)

        # Update status (during stabilization)
        now_t = time.time()
        if status_path and (now_t - last_status_t) >= STATUS_MIN_PERIOD_S:
            last_status_t = now_t
            status_state.update({
                "phase": "stabilizing",
                "last_heading_deg": float(yaw),
                "stabilization_samples": len(recent_yaws),
                "stabilization_required": STABILIZATION_SAMPLES,
                "stability_progress": int((consecutive_stable / max(1, required_stable)) * 100),
                "ts_local": datetime.now().isoformat(timespec="milliseconds"),
            })
            atomic_write_json(status_path, status_state)

        if len(recent_yaws) < STABILIZATION_SAMPLES:
            if not collecting_msg:
                print("[INFO] Collecting initial samples...")
                collecting_msg = True
            time.sleep(1.0 / SAMPLE_RATE)
            continue

        mean_yaw, std_dev = circular_mean_std_deg(list(recent_yaws))

        if std_dev < STABILITY_THRESHOLD:
            consecutive_stable += 1
        else:
            consecutive_stable = 0

        if consecutive_stable >= required_stable:
            print(f"\n[OK] Stabilized. Yaw reference = {mean_yaw:.2f}°")
            return float(mean_yaw)

        if consecutive_stable > 0:
            progress = (consecutive_stable / required_stable) * 100
            print(f"[INFO] Confirming stability... {int(progress):3d}% | std={std_dev:.4f}°", end="\r")
        else:
            print(f"[INFO] Stabilizing... yaw={yaw:6.2f}° | std={std_dev:.4f}°", end="\r")

        time.sleep(1.0 / SAMPLE_RATE)


# ==========================================================
# MAIN
# ==========================================================
def main():
    ap = argparse.ArgumentParser(description="BNO055 IMU logger (CSV + optional status JSON)")
    ap.add_argument("--outdir", default=".", help="Output directory for CSV")
    ap.add_argument("--status", default="", help="Path to status JSON file (optional)")
    args = ap.parse_args()

    outdir = Path(args.outdir).expanduser().resolve()
    outdir.mkdir(parents=True, exist_ok=True)

    status_path = Path(args.status).expanduser().resolve() if args.status else None

    print("[INFO] Starting BNO055 logger...")

    # Initial status structure (for run_all_loggers)
    status_state = {
        "ts_local": datetime.now().isoformat(timespec="milliseconds"),
        "running": True,
        "started": False,                 # becomes True after CSV logging begins
        "phase": "init",
        "csv_name": "",
        "outdir": str(outdir),

        "sample_rate_hz": float(SAMPLE_RATE),
        "yaw_ref_deg": None,

        "samples": 0,

        # calibration snapshot (updates during logging)
        "cal_sys": 0, "cal_g": 0, "cal_a": 0, "cal_m": 0,

        # last values (optional, for UI)
        "last_heading_deg": None,
        "last_roll_deg": None,
        "last_pitch_deg": None,
        "last_yaw_diff_deg": None,
        "last_temp_c": None,
    }
    if status_path:
        atomic_write_json(status_path, status_state)

    # Require calibration before accessing the sensor.
    offsets_path = Path(__file__).resolve().parent / OFFSETS_FILE
    offsets_from_file = load_offsets_from_json(offsets_path)

    # Init I2C + BNO055
    i2c = board.I2C()
    sensor = adafruit_bno055.BNO055_I2C(i2c)

    # Set mode
    try:
        sensor.mode = adafruit_bno055.NDOF_MODE if USE_NDOF_MODE else adafruit_bno055.IMUPLUS_MODE
        time.sleep(0.5)
        print(f"[INFO] Sensor mode set to: {sensor.mode}")
    except Exception as e:
        print(f"[WARN] Could not set mode: {e}")

    # Apply offsets
    apply_offsets(sensor, offsets_from_file)
    time.sleep(1.0)

    magnetic_enabled = sensor.mode == adafruit_bno055.NDOF_MODE
    sample_buffer = bytearray(46)

    # Stabilize + reference yaw
    status_state["phase"] = "stabilizing"
    if status_path:
        atomic_write_json(status_path, status_state)

    yaw_ref = stabilize_and_get_yaw_reference(sensor, status_path, status_state)
    status_state["yaw_ref_deg"] = float(yaw_ref)

    # Prepare output file
    agora = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{LOG_BASENAME}_{agora}.csv"
    filepath = outdir / filename

    fieldnames = [
        "timestamp",
        "heading_deg", "roll_deg", "pitch_deg",
        "yaw_diff_deg",
        "cal_sys", "cal_g", "cal_a", "cal_m",
        "temp_c",
    ]
    fieldnames.extend(column for _, columns in SENSOR_VECTORS for column in columns)

    print(f"\n[INFO] Logging to {filename} (Ctrl+C to stop)")
    period = 1.0 / SAMPLE_RATE
    samples = 0

    # Mark started for orchestrator ONLY when we really begin writing CSV
    status_state.update({
        "phase": "logging",
        "started": True,
        "csv_name": filename,
        "ts_local": datetime.now().isoformat(timespec="milliseconds"),
    })
    if status_path:
        atomic_write_json(status_path, status_state)

    last_status_t = 0.0

    with open(filepath, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        f.flush()

        try:
            while True:
                t0 = time.monotonic()

                try:
                    euler, calibration, temp_c, sensor_vectors = read_sample(
                        sensor.i2c_device, sample_buffer, magnetic_enabled
                    )
                except OSError:
                    # Keep the timeline and leave unavailable measurements blank.
                    euler = (None, None, None)
                    calibration = (0, 0, 0, 0)
                    temp_c = None
                    sensor_vectors = {}

                timestamp = datetime.now().isoformat(timespec="milliseconds")
                heading_deg, roll_deg, pitch_deg = euler
                yaw_diff = angle_diff_deg(heading_deg, yaw_ref) if heading_deg is not None else None
                sys_c, g_c, a_c, m_c = calibration
                writer.writerow({
                    "timestamp": timestamp,
                    "heading_deg": heading_deg if heading_deg is not None else "",
                    "roll_deg": roll_deg if roll_deg is not None else "",
                    "pitch_deg": pitch_deg if pitch_deg is not None else "",
                    "yaw_diff_deg": yaw_diff if yaw_diff is not None else "",
                    "cal_sys": sys_c,
                    "cal_g": g_c,
                    "cal_a": a_c,
                    "cal_m": m_c,
                    "temp_c": temp_c if temp_c is not None else "",
                    **sensor_vectors,
                })
                samples += 1

                if samples % FLUSH_EVERY_SAMPLES == 0:
                    f.flush()

                # Status JSON update (throttled)
                now_t = time.time()
                if status_path and (now_t - last_status_t) >= STATUS_MIN_PERIOD_S:
                    last_status_t = now_t
                    status_state.update({
                        "ts_local": datetime.now().isoformat(timespec="milliseconds"),
                        "samples": int(samples),
                        "cal_sys": int(sys_c), "cal_g": int(g_c), "cal_a": int(a_c), "cal_m": int(m_c),
                        "last_heading_deg": float(heading_deg) if heading_deg is not None else None,
                        "last_roll_deg": float(roll_deg) if roll_deg is not None else None,
                        "last_pitch_deg": float(pitch_deg) if pitch_deg is not None else None,
                        "last_yaw_diff_deg": float(yaw_diff) if yaw_diff is not None else None,
                        "last_temp_c": float(temp_c) if temp_c is not None else None,
                    })
                    atomic_write_json(status_path, status_state)

                elapsed = time.monotonic() - t0
                to_sleep = period - elapsed
                if to_sleep > 0:
                    time.sleep(to_sleep)

        except KeyboardInterrupt:
            print("\n[INFO] Stopping...")

    print(f"[OK] Saved: {filename} | samples={samples}")

    # Final status
    if status_path:
        status_state.update({
            "ts_local": datetime.now().isoformat(timespec="milliseconds"),
            "running": False,
            "started": False,
            "phase": "stopped",
            "samples": int(samples),
        })
        atomic_write_json(status_path, status_state)

if __name__ == "__main__":
    main()
