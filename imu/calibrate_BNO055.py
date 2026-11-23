#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# SPDX-FileCopyrightText: 2023 JG for Cedar Grove Maker Studios
# SPDX-License-Identifier: MIT

"""
BNO055 calibration helper.

Default behavior:
- calibrates the magnetometer, accelerometer, and gyroscope
- contains no preset sensor-specific offsets
- saves all offsets to bno055_offsets.json

If only_magnetometer = False, it will also calibrate accelerometer and gyroscope.
"""

import json
import time
from pathlib import Path

import board
import adafruit_bno055


class Mode:
    CONFIG_MODE = 0x00
    ACCONLY_MODE = 0x01
    MAGONLY_MODE = 0x02
    GYRONLY_MODE = 0x03
    ACCMAG_MODE = 0x04
    ACCGYRO_MODE = 0x05
    MAGGYRO_MODE = 0x06
    AMG_MODE = 0x07
    IMUPLUS_MODE = 0x08
    COMPASS_MODE = 0x09
    M4G_MODE = 0x0A
    NDOF_FMC_OFF_MODE = 0x0B
    NDOF_MODE = 0x0C


# ==========================================================
# USER CONFIG
# ==========================================================
only_magnetometer = False

# Keep these fixed when only_magnetometer = False
OFFSETS_GYROSCOPE = None
OFFSETS_ACCELEROMETER = None

# Optional: if you already have an old magnetometer offset and want to preload it
# before recalibration, set it here. Otherwise leave as None.
PRESET_MAGNETOMETER = None

# Output file
OFFSETS_JSON = "bno055_offsets.json"

# ==========================================================
# HELPERS
# ==========================================================
def calib_tuple(sensor):
    """Return (sys, gyro, accel, mag) as ints."""
    try:
        st = sensor.calibration_status
        if st is None or len(st) != 4:
            return (0, 0, 0, 0)
        return tuple(int(x) if x is not None else 0 for x in st)
    except Exception:
        return (0, 0, 0, 0)


def print_status_line(prefix=""):
    sys_c, gyr_c, acc_c, mag_c = calib_tuple(sensor)
    print(
        f"{prefix}SYS={sys_c}/3 GYR={gyr_c}/3 ACC={acc_c}/3 MAG={mag_c}/3"
    )

def save_offsets_json(
    out_path: Path,
    mag_offsets,
    gyro_offsets,
    accel_offsets,
    only_mag_flag: bool,
):
    data = {
        "magnetometer": list(mag_offsets) if mag_offsets is not None else None,
        "gyroscope": list(gyro_offsets) if gyro_offsets is not None else None,
        "accelerometer": list(accel_offsets) if accel_offsets is not None else None,
        "only_magnetometer": bool(only_mag_flag),
        "saved_at_epoch": time.time(),
    }

    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

    return data


# ==========================================================
# SENSOR INIT
# ==========================================================
# Uncomment these lines for UART interface connection
# uart = board.UART()
# sensor = adafruit_bno055.BNO055_UART(uart)

# I2C interface connection
i2c = board.I2C()  # For board.SCL and board.SDA
sensor = adafruit_bno055.BNO055_I2C(i2c)

# Set sensor mode
sensor.mode = Mode.NDOF_MODE
time.sleep(0.5)

print("[INFO] BNO055 connected.")
print(f"[INFO] Sensor mode set to NDOF_MODE ({sensor.mode}).")

# Apply fixed offsets you want to preserve
try:
    if OFFSETS_GYROSCOPE is not None:
        sensor.offsets_gyroscope = OFFSETS_GYROSCOPE
    if OFFSETS_ACCELEROMETER is not None:
        sensor.offsets_accelerometer = OFFSETS_ACCELEROMETER
    if PRESET_MAGNETOMETER is not None:
        sensor.offsets_magnetometer = PRESET_MAGNETOMETER
    print("[INFO] Preset offsets applied.")
    print(f"       GYR={OFFSETS_GYROSCOPE}")
    print(f"       ACC={OFFSETS_ACCELEROMETER}")
    print(f"       MAG={PRESET_MAGNETOMETER}")
except Exception as e:
    print(f"[WARN] Could not apply preset offsets: {e}")

time.sleep(1.0)
print_status_line("[INFO] Initial calibration status: ")


# ==========================================================
# CALIBRATION
# ==========================================================
print("\nMagnetometer: perform the figure-eight calibration dance.")
print("Move the IMU slowly in different orientations and away from magnetic interference.")

while True:
    sys_c, gyr_c, acc_c, mag_c = calib_tuple(sensor)
    print(f"Mag Calib Status: {100 / 3 * mag_c:3.0f}%   (SYS={sys_c} GYR={gyr_c} ACC={acc_c} MAG={mag_c})")
    if mag_c == 3:
        break
    time.sleep(1.0)

print("... MAGNETOMETER CALIBRATED")
time.sleep(1.0)

if not only_magnetometer:
    print("\nAccelerometer: perform the six-step calibration routine.")
    print("Place sensor in six stable orientations until ACC reaches 3.")
    time.sleep(2.0)

    while True:
        sys_c, gyr_c, acc_c, mag_c = calib_tuple(sensor)
        print(f"Accel Calib Status: {100 / 3 * acc_c:3.0f}%   (SYS={sys_c} GYR={gyr_c} ACC={acc_c} MAG={mag_c})")
        if acc_c == 3:
            break
        time.sleep(1.0)

    print("... ACCELEROMETER CALIBRATED")
    time.sleep(1.0)

    print("\nGyroscope: keep the IMU still for a few seconds.")
    time.sleep(2.0)

    while True:
        sys_c, gyr_c, acc_c, mag_c = calib_tuple(sensor)
        print(f"Gyro Calib Status: {100 / 3 * gyr_c:3.0f}%   (SYS={sys_c} GYR={gyr_c} ACC={acc_c} MAG={mag_c})")
        if gyr_c == 3:
            break
        time.sleep(1.0)

    print("... GYROSCOPE CALIBRATED")
    time.sleep(1.0)


# ==========================================================
# FINAL OFFSETS
# ==========================================================
mag_offsets = sensor.offsets_magnetometer

if mag_offsets is None:
    raise RuntimeError("Magnetometer calibration finished, but no magnetometer offsets were read from the sensor.")

if only_magnetometer:
    gyro_offsets = OFFSETS_GYROSCOPE
    accel_offsets = OFFSETS_ACCELEROMETER
else:
    gyro_offsets = sensor.offsets_gyroscope
    accel_offsets = sensor.offsets_accelerometer

if any(value is None for value in (mag_offsets, gyro_offsets, accel_offsets)):
    raise RuntimeError("Full calibration offsets are required. Run with only_magnetometer = False.")

print("\nCALIBRATION COMPLETED")
print("Final calibration status:")
print_status_line("  ")

print("\nOffsets to use:")
print(f"  Offsets_Magnetometer:  {mag_offsets}")
print(f"  Offsets_Gyroscope:     {gyro_offsets}")
print(f"  Offsets_Accelerometer: {accel_offsets}")

out_path = Path(__file__).resolve().parent / OFFSETS_JSON
data = save_offsets_json(
    out_path=out_path,
    mag_offsets=mag_offsets,
    gyro_offsets=gyro_offsets,
    accel_offsets=accel_offsets,
    only_mag_flag=only_magnetometer,
)

print(f"\n[INFO] Offsets saved to: {out_path}")
