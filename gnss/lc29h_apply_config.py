#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
lc29h_apply_config.py

LC29H (DA) configuration helper with:
  - rover profile
  - base-survey profile
  - base-fixed profile using lat/lon/height JSON -> WGS84 ECEF X/Y/Z
  - NMEA decimal-place precision configuration
  - NMEA precision query and live GGA/RMC verification

Examples:

  # Query current state
  python3 lc29h_apply_config.py --profile query-only

  # Rover mode
  python3 lc29h_apply_config.py --profile rover --save --cold

  # Base mode with survey-in/self-survey
  # Min duration and accuracy affect survey-in behavior.
  python3 lc29h_apply_config.py --profile base-survey --svin-min-dur 600 --svin-acc-limit 1.0 --save --cold

  # Base mode with fixed coordinates read from JSON
  python3 lc29h_apply_config.py --profile base-fixed --fixed-json base_fixed_position.json --save --cold

  # Set NMEA position precision to 8 decimal places and altitude to 3
  python3 lc29h_apply_config.py --profile precision --pos-dp 8 --alt-dp 3 --save --cold

  # Query configured precision and inspect live GGA/RMC output
  python3 lc29h_apply_config.py --profile precision-check --precision-samples 8

Expected fixed JSON formats accepted:

  {
    "latitude_deg": -22.5264,
    "longitude_deg": -42.9748,
    "height_hae_m": 23.0
  }

or:

  {
    "fixed_position": {
      "latitude_deg": -22.5264,
      "longitude_deg": -42.9748,
      "height_hae_m": 23.0
    }
  }

or:

  {
    "final_position": {
      "latitude_deg": -22.5264,
      "longitude_deg": -42.9748,
      "height_hae_m": 23.0
    }
  }

Notes:
- PQTMCFGRCVRMODE:
    1 = Rover
    2 = Base station
- PQTMCFGSVIN:
    mode 1 = survey-in
    mode 2 = fixed, ECEF X/Y/Z in meters
- PAIR432 = RTCM output mode
- PAIR434 = RTCM 1005 antenna reference point output
- PAIR436 = RTCM ephemeris output
- PQTMCFGNMEADP:
    POS_DP range 0-8; default is normally 6
    ALT_DP range 0-3
"""

import argparse
import json
import math
import os
import serial
import time
from datetime import datetime
from pathlib import Path


# ==========================================================
# NMEA / SERIAL HELPERS
# ==========================================================
def add_checksum(sentence_no_dollar: str) -> str:
    cs = 0
    for ch in sentence_no_dollar:
        cs ^= ord(ch)
    return f"${sentence_no_dollar}*{cs:02X}"


def send_command(ser: serial.Serial, cmd_no_checksum: str, read_time: float = 1.0):
    full_cmd = add_checksum(cmd_no_checksum)

    ser.reset_input_buffer()
    ser.write((full_cmd + "\r\n").encode("ascii"))
    ser.flush()

    responses = []
    t0 = time.time()

    while time.time() - t0 < read_time:
        line = ser.readline()
        if not line:
            continue
        try:
            text = line.decode("ascii", errors="ignore").strip()
        except Exception:
            continue
        if text:
            responses.append(text)

    return full_cmd, responses


def log_block(fp, title, sent_cmd, responses):
    fp.write(f"\n=== {title} ===\n")
    fp.write(f"SENT: {sent_cmd}\n")
    if responses:
        for r in responses:
            fp.write(r + "\n")
    else:
        fp.write("(no response)\n")


def fractional_digits(value: str) -> int:
    """Return the number of digits after the decimal point."""
    value = (value or "").strip()
    if "." not in value:
        return 0
    return len(value.split(".", 1)[1])


def parse_nmea_position_precision(line: str):
    """
    Inspect a GGA or RMC sentence and return the decimal-place count of the
    latitude and longitude fields as transmitted by the receiver.

    The count refers to decimal places in NMEA minutes:
      latitude  ddmm.mmmmmmmm
      longitude dddmm.mmmmmmmm
    """
    if not line.startswith("$"):
        return None

    body = line.split("*", 1)[0]
    fields = body.split(",")
    if not fields:
        return None

    sentence_type = fields[0][-3:]

    if sentence_type == "GGA" and len(fields) > 5:
        lat_field = fields[2]
        lon_field = fields[4]
    elif sentence_type == "RMC" and len(fields) > 6:
        lat_field = fields[3]
        lon_field = fields[5]
    else:
        return None

    if not lat_field or not lon_field:
        return None

    return {
        "sentence_type": sentence_type,
        "latitude_field": lat_field,
        "longitude_field": lon_field,
        "latitude_dp": fractional_digits(lat_field),
        "longitude_dp": fractional_digits(lon_field),
        "raw": line,
    }


def collect_live_precision_samples(
    ser: serial.Serial,
    requested_samples: int = 5,
    timeout_s: float = 12.0,
    initial_lines=None,
):
    """
    Collect GGA/RMC sentences and determine the position decimal places
    actually emitted by the receiver.
    """
    samples = []
    seen = set()

    def consider(line: str):
        info = parse_nmea_position_precision(line)
        if info is None:
            return
        key = info["raw"]
        if key not in seen:
            seen.add(key)
            samples.append(info)

    for line in initial_lines or []:
        consider(line)

    t0 = time.time()
    while len(samples) < requested_samples and time.time() - t0 < timeout_s:
        raw = ser.readline()
        if not raw:
            continue
        try:
            line = raw.decode("ascii", errors="ignore").strip()
        except Exception:
            continue
        if line:
            consider(line)

    return samples


def validate_precision_args(utc_dp, pos_dp, alt_dp, dop_dp, spd_dp, cog_dp):
    limits = {
        "utc_dp": (utc_dp, 0, 3),
        "pos_dp": (pos_dp, 0, 8),
        "alt_dp": (alt_dp, 0, 3),
        "dop_dp": (dop_dp, 0, 3),
        "spd_dp": (spd_dp, 0, 3),
        "cog_dp": (cog_dp, 0, 3),
    }

    for name, (value, low, high) in limits.items():
        if not low <= value <= high:
            raise ValueError(f"{name} must be between {low} and {high}; got {value}.")


def get_precision_write_command(utc_dp, pos_dp, alt_dp, dop_dp, spd_dp, cog_dp):
    validate_precision_args(utc_dp, pos_dp, alt_dp, dop_dp, spd_dp, cog_dp)
    return (
        f"PQTMCFGNMEADP,W,{utc_dp},{pos_dp},{alt_dp},"
        f"{dop_dp},{spd_dp},{cog_dp}"
    )


# ==========================================================
# GEODESY
# ==========================================================
def geodetic_to_ecef_wgs84(lat_deg: float, lon_deg: float, h_m: float):
    """
    Convert WGS84 geodetic latitude/longitude/ellipsoidal height to ECEF.

    h_m must be HAE / ellipsoidal height, not MSL.
    """
    a = 6378137.0
    f = 1.0 / 298.257223563
    e2 = f * (2.0 - f)

    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    N = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)

    x = (N + h_m) * cos_lat * cos_lon
    y = (N + h_m) * cos_lat * sin_lon
    z = (N * (1.0 - e2) + h_m) * sin_lat

    return x, y, z


def _nested_get(d, keys):
    cur = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return None
        cur = cur[k]
    return cur


def load_fixed_position_json(path: Path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    candidates = [
        data,
        data.get("fixed_position") if isinstance(data, dict) else None,
        data.get("final_position") if isinstance(data, dict) else None,
        data.get("base_position") if isinstance(data, dict) else None,
        data.get("position") if isinstance(data, dict) else None,
    ]

    for obj in candidates:
        if not isinstance(obj, dict):
            continue

        lat = obj.get("latitude_deg", obj.get("lat_deg", obj.get("lat")))
        lon = obj.get("longitude_deg", obj.get("lon_deg", obj.get("lon")))
        h = obj.get("height_hae_m", obj.get("hae_m", obj.get("height_m", obj.get("altitude_m"))))

        if lat is not None and lon is not None and h is not None:
            return float(lat), float(lon), float(h), data

    raise ValueError(
        "Could not find latitude_deg, longitude_deg, height_hae_m in JSON. "
        "Accepted keys: latitude_deg/longitude_deg/height_hae_m, lat/lon/hae_m."
    )


# ==========================================================
# COMMAND PROFILES
# ==========================================================
def common_nmea_monitor_commands():
    return [
        "PQTMCFGMSGRATE,W,GGA,1",
        "PQTMCFGMSGRATE,W,RMC,1",
        "PQTMCFGMSGRATE,W,GSA,1",
        "PQTMCFGMSGRATE,W,GSV,0",
    ]


def get_rover_write_commands():
    return [
        "PQTMCFGRCVRMODE,W,1",      # Rover
        "PQTMCFGSVIN,W,0,0,0,0,0,0", # Disable base survey/fixed mode
        "PAIR432,1",                # RTCM output = MSM7, if rover raw obs are desired
        "PAIR434,0",                # 1005 off
        "PAIR436,1",                # ephemerides on
    ] + common_nmea_monitor_commands()


def get_base_survey_write_commands(min_dur_s: int, acc_limit_m: float):
    return [
        "PQTMCFGRCVRMODE,W,2",                         # Base station
        f"PQTMCFGSVIN,W,1,{int(min_dur_s)},{float(acc_limit_m):.3f},0,0,0",  # Survey-in
        "PAIR432,1",                                   # RTCM output = MSM7
        "PAIR434,1",                                   # 1005 on
        "PAIR436,1",                                   # ephemerides on
    ] + common_nmea_monitor_commands()


def get_base_fixed_write_commands(fixed_json_path: Path):
    lat, lon, h, raw_json = load_fixed_position_json(fixed_json_path)
    x, y, z = geodetic_to_ecef_wgs84(lat, lon, h)

    fixed_info = {
        "source_json": str(fixed_json_path),
        "latitude_deg": lat,
        "longitude_deg": lon,
        "height_hae_m": h,
        "ecef_x_m": x,
        "ecef_y_m": y,
        "ecef_z_m": z,
    }

    cmds = [
        "PQTMCFGRCVRMODE,W,2",                                  # Base station
        f"PQTMCFGSVIN,W,2,0,0,{x:.4f},{y:.4f},{z:.4f}",          # Fixed ECEF ARP
        "PAIR432,1",                                            # RTCM output = MSM7
        "PAIR434,1",                                            # 1005 on
        "PAIR436,1",                                            # ephemerides on
    ] + common_nmea_monitor_commands()

    return cmds, fixed_info


def get_query_commands():
    return [
        "PQTMVERNO",
        "PQTMCFGRCVRMODE,R",
        "PQTMCFGSVIN,R",
        "PQTMSVINSTATUS",
        "PAIR865,0,0",
        "PAIR867,0,0",
        "PAIR433",
        "PAIR435",
        "PAIR437",
        "PQTMCFGMSGRATE,R,GGA",
        "PQTMCFGMSGRATE,R,RMC",
        "PQTMCFGMSGRATE,R,GSA",
        "PQTMCFGMSGRATE,R,GSV",
        "PQTMCFGNMEADP,R",
    ]


# ==========================================================
# MAIN
# ==========================================================
def main():
    ap = argparse.ArgumentParser(
        description="Configure LC29H DA profiles, including base survey-in and fixed-coordinate base."
    )

    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=115200)

    ap.add_argument(
        "--profile",
        choices=["rover", "base-survey", "base-fixed", "precision", "precision-check", "query-only"],
        default="query-only",
    )

    ap.add_argument("--outdir", default=".")
    ap.add_argument("--save", action="store_true", help="send PQTMSAVEPAR at the end")
    ap.add_argument("--cold", action="store_true", help="send PQTMCOLD at the end")
    ap.add_argument("--delay", type=float, default=0.3, help="delay between commands")
    ap.add_argument("--read-time", type=float, default=1.0, help="read time after each command")

    ap.add_argument("--svin-min-dur", type=int, default=600, help="survey-in minimum duration in seconds")
    ap.add_argument("--svin-acc-limit", type=float, default=1.0, help="survey-in 3D accuracy limit in meters")

    ap.add_argument("--fixed-json", default="base_fixed_position.json", help="JSON with fixed lat/lon/height_hae_m")

    # NMEA decimal-place configuration (PQTMCFGNMEADP).
    ap.add_argument("--utc-dp", type=int, default=3, help="UTC fractional-second decimals, range 0-3")
    ap.add_argument("--pos-dp", type=int, default=8, help="latitude/longitude minute decimals, range 0-8")
    ap.add_argument("--alt-dp", type=int, default=3, help="altitude/geoid decimals, range 0-3")
    ap.add_argument("--dop-dp", type=int, default=2, help="DOP decimals, range 0-3")
    ap.add_argument("--spd-dp", type=int, default=3, help="speed decimals, range 0-3")
    ap.add_argument("--cog-dp", type=int, default=2, help="course-over-ground decimals, range 0-3")

    ap.add_argument(
        "--precision-samples",
        type=int,
        default=6,
        help="number of live GGA/RMC sentences inspected in precision-check mode",
    )
    ap.add_argument(
        "--precision-timeout",
        type=float,
        default=15.0,
        help="maximum seconds spent waiting for live GGA/RMC precision samples",
    )

    args = ap.parse_args()

    validate_precision_args(
        args.utc_dp,
        args.pos_dp,
        args.alt_dp,
        args.dop_dp,
        args.spd_dp,
        args.cog_dp,
    )

    if args.precision_samples < 1:
        ap.error("--precision-samples must be at least 1")
    if args.precision_timeout <= 0:
        ap.error("--precision-timeout must be greater than zero")

    outdir = Path(args.outdir).expanduser().resolve()
    outdir.mkdir(parents=True, exist_ok=True)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    logfile = outdir / f"lc29h_apply_{args.profile}_{ts}.txt"

    fixed_info = None

    if args.profile == "rover":
        write_cmds = get_rover_write_commands()
    elif args.profile == "base-survey":
        write_cmds = get_base_survey_write_commands(args.svin_min_dur, args.svin_acc_limit)
    elif args.profile == "base-fixed":
        write_cmds, fixed_info = get_base_fixed_write_commands(Path(args.fixed_json).expanduser().resolve())
    elif args.profile == "precision":
        write_cmds = [
            get_precision_write_command(
                args.utc_dp,
                args.pos_dp,
                args.alt_dp,
                args.dop_dp,
                args.spd_dp,
                args.cog_dp,
            )
        ]
    else:
        # query-only and precision-check do not change the receiver.
        write_cmds = []

    print(f"[INFO] Log: {logfile}")

    if fixed_info:
        print("[INFO] Fixed base coordinates loaded:")
        print(f"       lat = {fixed_info['latitude_deg']:.12f} deg")
        print(f"       lon = {fixed_info['longitude_deg']:.12f} deg")
        print(f"       HAE = {fixed_info['height_hae_m']:.4f} m")
        print("[INFO] Converted to WGS84 ECEF:")
        print(f"       X = {fixed_info['ecef_x_m']:.4f} m")
        print(f"       Y = {fixed_info['ecef_y_m']:.4f} m")
        print(f"       Z = {fixed_info['ecef_z_m']:.4f} m")

    ser = serial.Serial(args.port, args.baud, timeout=0.2)

    try:
        with open(logfile, "w", encoding="utf-8") as fp:
            fp.write(f"Port: {args.port}\n")
            fp.write(f"Baud: {args.baud}\n")
            fp.write(f"Profile: {args.profile}\n")
            fp.write(f"Save: {args.save}\n")
            fp.write(f"Cold: {args.cold}\n")
            fp.write(f"Timestamp: {datetime.now().isoformat()}\n")
            fp.write(f"SurveyInMinDur_s: {args.svin_min_dur}\n")
            fp.write(f"SurveyInAccLimit_m: {args.svin_acc_limit}\n")
            fp.write(f"FixedJson: {args.fixed_json}\n")
            fp.write(
                "RequestedNmeaDP: "
                f"UTC={args.utc_dp}, POS={args.pos_dp}, ALT={args.alt_dp}, "
                f"DOP={args.dop_dp}, SPD={args.spd_dp}, COG={args.cog_dp}\n"
            )

            if fixed_info:
                fp.write("\n=== FIXED POSITION INPUT ===\n")
                fp.write(json.dumps(fixed_info, indent=2) + "\n")

            if write_cmds:
                print(f"[INFO] Applying profile: {args.profile}")
                for cmd in write_cmds:
                    sent, resp = send_command(ser, cmd, read_time=args.read_time)
                    print(f"[SEND] {sent}")
                    log_block(fp, f"WRITE {cmd}", sent, resp)
                    time.sleep(args.delay)
            else:
                print(f"[INFO] {args.profile} mode. No write commands sent.")

            if args.save:
                cmd = "PQTMSAVEPAR"
                sent, resp = send_command(ser, cmd, read_time=1.5)
                print(f"[SEND] {sent}")
                log_block(fp, "SAVE", sent, resp)
                time.sleep(0.5)

            if args.cold:
                cmd = "PQTMCOLD"
                sent, resp = send_command(ser, cmd, read_time=1.0)
                print(f"[SEND] {sent}")
                log_block(fp, "COLD RESTART", sent, resp)
                print("[INFO] Waiting 5 s after cold restart...")
                time.sleep(5.0)

            if args.profile == "precision-check":
                print("[INFO] Querying configured NMEA decimal places...")
                sent, resp = send_command(
                    ser,
                    "PQTMCFGNMEADP,R",
                    read_time=args.read_time,
                )
                print(f"[QUERY] {sent}")
                log_block(fp, "PRECISION CONFIG QUERY", sent, resp)

                print(
                    "[INFO] Inspecting live GGA/RMC fields "
                    f"(target: {args.precision_samples} sentences)..."
                )
                samples = collect_live_precision_samples(
                    ser,
                    requested_samples=args.precision_samples,
                    timeout_s=args.precision_timeout,
                    initial_lines=resp,
                )

                fp.write("\n=== LIVE NMEA POSITION PRECISION CHECK ===\n")
                if samples:
                    lat_dps = sorted({s["latitude_dp"] for s in samples})
                    lon_dps = sorted({s["longitude_dp"] for s in samples})

                    print(
                        "[CHECK] Observed latitude decimal places in NMEA minutes: "
                        f"{lat_dps}"
                    )
                    print(
                        "[CHECK] Observed longitude decimal places in NMEA minutes: "
                        f"{lon_dps}"
                    )

                    fp.write(f"SamplesCaptured: {len(samples)}\n")
                    fp.write(f"LatitudeDPObserved: {lat_dps}\n")
                    fp.write(f"LongitudeDPObserved: {lon_dps}\n")

                    for s in samples:
                        fp.write(
                            f"{s['sentence_type']}: "
                            f"lat={s['latitude_field']} ({s['latitude_dp']} dp), "
                            f"lon={s['longitude_field']} ({s['longitude_dp']} dp)\n"
                        )
                        fp.write(s["raw"] + "\n")

                    if lat_dps == [args.pos_dp] and lon_dps == [args.pos_dp]:
                        print(
                            f"[PASS] Live NMEA position fields use {args.pos_dp} "
                            "decimal places."
                        )
                        fp.write("PrecisionCheck: PASS\n")
                    else:
                        print(
                            "[WARN] Live position precision does not match "
                            f"--pos-dp {args.pos_dp}."
                        )
                        fp.write("PrecisionCheck: MISMATCH\n")
                else:
                    print(
                        "[WARN] No valid GGA/RMC position sentences were captured. "
                        "Ensure GGA or RMC output is enabled."
                    )
                    fp.write("(no valid GGA/RMC position sentences captured)\n")
                    fp.write("PrecisionCheck: NO_DATA\n")

            print("[INFO] Running verification queries...")
            for cmd in get_query_commands():
                sent, resp = send_command(ser, cmd, read_time=args.read_time)
                print(f"[QUERY] {sent}")
                log_block(fp, f"QUERY {cmd}", sent, resp)
                time.sleep(0.2)

        print("[DONE] Finished.")

    finally:
        ser.close()


if __name__ == "__main__":
    main()
