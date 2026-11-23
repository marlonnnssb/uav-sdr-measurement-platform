#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ntrip_rover_ibge.py — LC29H rover acquisition with IBGE NTRIP corrections.

Requirements:
    python3 -m pip install pyserial
    Configure the receiver separately with lc29h_apply_config.py.
    
Raspberry Pi prerequisites:
    Enable the UART interface.
    Disable the serial login console on the UART used by the receiver.
    Ensure the user has permission to access the serial device.

The logger opens /dev/ttyAMA0 at 115200 baud by default.
It does not modify Raspberry Pi OS settings or the receiver's baud rate.

Before running:

    Edit IBGE_USERNAME and IBGE_PASSWORD below using your IBGE account.
    The script stops before hardware access or logging if either is unchanged.

Usage:
    python3 ntrip_rover_ibge.py --mount RJNI0 --outdir logs --indicator

Outputs:
    rtkoutput_<timestamp>_svc=ibge.csv : positioning and quality measurements.
    rover_<timestamp>.rtcm3 : CRC-checked RTCM frames OUTPUT BY THE ROVER UART,
        for subsequent PPK processing. This is not a log of IBGE corrections.
        The file may be empty if the receiver does not output RTCM observations.
    Optional status JSON via --status.

The IBGE host and port are fixed to gps-ntrip.ibge.gov.br:2101.
Only the IBGE station mountpoint can be selected; no local-base profile is included.
During connection outages, GNSS/RTCM logging continues and connection is retried.
CSV Altitude is ellipsoidal height: GGA mean-sea-level altitude + geoid separation.
This script does not configure the receiver, solve PPK, or guarantee an RTK fix.
Stop with Ctrl+C.
"""

import argparse, base64, socket, threading, time, csv, os, json
from pathlib import Path
from datetime import datetime, timezone
import serial  # pip install pyserial
from collections import Counter

# ==========================================================
# REQUIRED LOCAL CONFIGURATION — replace BOTH placeholders.
# Never upload the edited credentials to GitHub.
# ==========================================================
IBGE_USERNAME = "YOUR_IBGE_USERNAME"
IBGE_PASSWORD = "YOUR_IBGE_PASSWORD"


def validate_ibge_credentials():
    """Reject missing or unchanged credentials without printing their values."""
    placeholders = {
        "YOUR_IBGE_USERNAME", "YOUR_IBGE_PASSWORD", "YOUR_USERNAME", "YOUR_PASSWORD",
        "USERNAME", "PASSWORD", "CHANGE_ME", "CHANGEME", "REPLACE_ME",
        "SEU_USUARIO", "SUA_SENHA", "<USERNAME>", "<PASSWORD>",
    }
    for name, value in (("IBGE_USERNAME", IBGE_USERNAME), ("IBGE_PASSWORD", IBGE_PASSWORD)):
        if not isinstance(value, str) or not value.strip() or value.strip().upper() in placeholders:
            raise ValueError(
                "Configure IBGE_USERNAME and IBGE_PASSWORD near the top of this script "
                "with your own IBGE credentials before running. "
                "No acquisition files have been created."
            )
    username = IBGE_USERNAME.strip()
    if ":" in username or any(ch in username + IBGE_PASSWORD for ch in "\r\n"):
        raise ValueError("Invalid IBGE credential format; check the configuration fields.")
    return username, IBGE_PASSWORD


CRC24Q_POLY = 0x1864CFB
CRC24Q_INIT = 0x000000

def crc24q(data: bytes) -> int:
    crc = CRC24Q_INIT
    for b in data:
        crc ^= (b << 16)
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= CRC24Q_POLY
        crc &= 0xFFFFFF
    return crc

def parse_rtcm_frames(buf: bytearray):
    """Yield (frame_bytes, msg_num) and consume from buf."""
    out = []
    i = 0
    while True:
        try:
            i = buf.index(0xD3, i)
        except ValueError:
            if len(buf) > 2:
                del buf[:-2]
            return out
        if i + 3 > len(buf):
            break
        length = ((buf[i+1] & 0x03) << 8) | buf[i+2]
        total = 3 + length + 3
        if i + total > len(buf):
            break
        frame = bytes(buf[i:i+total])
        calc = crc24q(frame[:-3])
        got = (frame[-3] << 16) | (frame[-2] << 8) | frame[-1]
        if calc == got:
            payload = frame[3:-3]
            msg_num = ((payload[0] << 4) | (payload[1] >> 4)) & 0x0FFF if len(payload) >= 2 else -1
            out.append((frame, msg_num))
            i += total
        else:
            i += 1
    del buf[:i]
    return out

def is_nmea_line(line: bytes) -> bool:
    return line.startswith(b"$") and 5 <= len(line) <= 200

def is_msm(mid: int) -> bool:
    # MSM families per constellation: 107x GPS, 108x GLO, 109x GAL, 111x QZSS, 112x BDS
    fam = mid // 10
    return fam in (107,108,109,111,112) and 1 <= (mid % 10) <= 7

EPH_LABEL = {1019:"GPS eph", 1020:"GLO eph", 1042:"BDS eph", 1046:"GAL eph"}

def summarize_counts(cnt: Counter):
    msm_ids = sorted(k for k in cnt if is_msm(k))
    eph_ids = sorted(k for k in cnt if k in EPH_LABEL)
    msm_str = " ".join(f"{k}:{cnt[k]}" for k in msm_ids) if msm_ids else "-"
    eph_str = " ".join(f"{k}:{cnt[k]}({EPH_LABEL[k]})" for k in eph_ids) if eph_ids else "-"
    return msm_str, eph_str


def utc_log(msg: str):
    print(f"[{datetime.now(timezone.utc).isoformat(timespec='seconds')}Z] {msg}", flush=True)

def atomic_write_json(path: Path, obj: dict):
    try:
        path = Path(path)
        tmp = path.with_suffix(path.suffix + ".tmp")
        with open(tmp, "w") as f:
            json.dump(obj, f, ensure_ascii=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except Exception:
        pass

def nmea_dm_to_deg(dm, hemi):
    if not dm:
        return None
    if isinstance(dm, (bytes, bytearray)):
        dm = dm.decode('ascii', 'ignore')
    if isinstance(hemi, (bytes, bytearray)):
        hemi = hemi.decode('ascii', 'ignore')
    dm = dm.strip()
    hemi = (hemi or "").strip().upper()
    if '.' not in dm:
        return None
    i = dm.find('.') - 2
    try:
        deg = int(dm[:i])
        minutes = float(dm[i:])
    except Exception:
        return None
    val = abs(deg + minutes / 60.0)
    if hemi in ('S', 'W'):
        val = -val
    return val

FIX_TEXT = {
    0:"INVALID", 1:"AUTONOMOUS", 2:"DGPS",
    4:"RTK_FIXED", 5:"RTK_FLOAT",
    6:"DR", 7:"MANUAL", 8:"SIM"
}

def parse_rmc_datetime_utc(fields):
    t = fields[1].decode('ascii','ignore') if len(fields) > 1 else ""
    d = fields[9].decode('ascii','ignore') if len(fields) > 9 else ""
    if len(t) < 6 or len(d) != 6:
        return None
    try:
        hh, mm, ss = int(t[0:2]), int(t[2:4]), int(t[4:6])
        micro = 0
        if '.' in t:
            micro = int((t.split('.', 1)[1] + "000000")[:6])
        dd, mo, yy = int(d[0:2]), int(d[2:4]), int(d[4:6])
        yyyy = 2000 + yy if yy < 80 else 1900 + yy
        return datetime(yyyy, mo, dd, hh, mm, ss, micro, tzinfo=timezone.utc)
    except Exception:
        return None

def combine_time_with_date(utc_time_bytes, dt_date):
    t = utc_time_bytes.decode('ascii','ignore') if isinstance(utc_time_bytes,(bytes,bytearray)) else (utc_time_bytes or "")
    if len(t) < 6:
        return None
    try:
        hh, mm, ss = int(t[0:2]), int(t[2:4]), int(t[4:6])
        micro = 0
        if '.' in t:
            micro = int((t.split('.', 1)[1] + "000000")[:6])
        return datetime(dt_date.year, dt_date.month, dt_date.day, hh, mm, ss, micro, tzinfo=timezone.utc)
    except Exception:
        return None

def iso_z(dt_utc):
    if dt_utc is None:
        return ""
    ms = int(round(dt_utc.microsecond / 1000.0))
    return dt_utc.replace(microsecond=ms * 1000).isoformat(timespec="milliseconds").replace("+00:00", "Z")

def cn0_summaries(state):
    all_vals = list(state.get("gsv_all_cno", {}).values())
    used_prns = state.get("gsa_used_prns", set()) or set()
    used_vals = [state["gsv_all_cno"][p] for p in used_prns if p in state.get("gsv_all_cno", {})]
    def avg(xs): return round(sum(xs)/len(xs), 1) if xs else None
    return (
        avg(used_vals), max(used_vals) if used_vals else None,
        avg(all_vals), max(all_vals) if all_vals else None
    )

def ntrip_connect(host, port, mount, user, passwd, timeout=10):
    auth = base64.b64encode(f"{user}:{passwd}".encode()).decode()
    req = (
        f"GET /{mount} HTTP/1.0\r\n"
        f"User-Agent: ntrip_rover/1.0\r\n"
        f"Accept: */*\r\n"
        f"Connection: keep-alive\r\n"
        f"Authorization: Basic {auth}\r\n\r\n"
    ).encode()

    s = socket.create_connection((host, int(port)), timeout=timeout)
    s.sendall(req)

    header = b""; s.settimeout(timeout)
    while b"\r\n\r\n" not in header:
        chunk = s.recv(4096)
        if not chunk:
            raise RuntimeError("Caster closed during header read.")
        header += chunk
        if len(header) > 8192:
            break

    body = header.split(b"\r\n\r\n", 1)[1] if b"\r\n\r\n" in header else b""
    text = header[: header.find(b"\r\n\r\n")].decode(errors="ignore")
    if not (text.startswith("ICY 200") or " 200 " in text):
        raise RuntimeError(f"NTRIP connection failed: {text.strip()}")
    s.settimeout(None)
    utc_log("NTRIP connected (200).")

    class SockWrap:
        def __init__(self, s, first):
            self.s = s
            self.first = first
            self.used = False
        def recv(self, n):
            if not self.used and self.first:
                self.used = True
                b = self.first
                self.first = b""
                return b
            return self.s.recv(n)
        def sendall(self, b):
            return self.s.sendall(b)
        def close(self):
            try:
                self.s.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            self.s.close()

    return SockWrap(s, body)
# Continue standalone logging when the IBGE correction connection fails.
def rtcm_downlink(sock, ser, link_down_evt, global_stop_evt, state):
    while not link_down_evt.is_set() and not global_stop_evt.is_set():
        try:
            data = sock.recv(4096)
            if not data:
                utc_log("Caster closed connection. Switching to standalone GNSS.")
                state["ntrip_mode"] = "STANDALONE"
                link_down_evt.set()
                break

            ser.write(data)

        except socket.timeout:
            utc_log("NTRIP recv timeout; keeping connection alive.")
            continue

        except OSError as e:
            utc_log(f"NTRIP socket OSError: {e}. Switching to standalone GNSS.")
            state["ntrip_mode"] = "STANDALONE"
            link_down_evt.set()
            break

        except Exception as e:
            utc_log(f"socket recv error: {e}")
            state["ntrip_mode"] = "STANDALONE"
            link_down_evt.set()
            break
# Continue standalone logging when the IBGE correction connection fails.
def gga_uplink(sock, latest_gga, link_down_evt, global_stop_evt, state, period=1.0):
    last = 0.0
    while not link_down_evt.is_set() and not global_stop_evt.is_set():
        try:
            now = time.time()
            if now - last >= period and latest_gga.get("bytes"):
                sock.sendall(latest_gga["bytes"])
                last = now
        except OSError as e:
            utc_log(f"GGA uplink OSError: {e}. Switching to standalone GNSS.")
            state["ntrip_mode"] = "STANDALONE"
            link_down_evt.set()
            break
        except Exception as e:
            utc_log(f"GGA send error: {e}")
            state["ntrip_mode"] = "STANDALONE"
            link_down_evt.set()
            break
        time.sleep(0.05)

def connect_with_retry_once(args, state):
    try:
        utc_log(f"Connecting to NTRIP {args.caster}:{args.port}/{args.mount} …")
        state["ntrip_mode"] = "NTRIP_CONNECTING"
        sock = ntrip_connect(args.caster, args.port, args.mount, args.user, args.password)
        state["ntrip_mode"] = "NTRIP_ACTIVE"
        return sock
    except Exception as e:
        utc_log(f"NTRIP unavailable ({e}). Staying in standalone GNSS.")
        state["ntrip_mode"] = "STANDALONE"
        return None

def write_csv_row(csv_writer, lat, lon, hae, alt_msl, dt_utc, state, status_path):
    if lat is None or lon is None:
        return

    timestamp_local = datetime.now().isoformat(timespec="milliseconds")
    timestamp_gps = iso_z(dt_utc) if dt_utc else ""

    fixq   = state.get("fix_quality")
    fix_tx = FIX_TEXT.get(fixq, None)
    nums   = state.get("num_sats")
    hdop   = state.get("hdop")
    vdop   = state.get("vdop")
    pdop   = state.get("pdop")
    age    = state.get("age_diff")
    refid  = state.get("ref_id")
    mode   = state.get("nav_mode")
    spd_kn = state.get("speed_kn")
    crs    = state.get("course_deg")

    cno_used_avg, cno_used_max, cno_all_avg, cno_all_max = cn0_summaries(state)
    state["cno_used_avg"], state["cno_used_max"], state["cno_all_avg"], state["cno_all_max"] = (
        cno_used_avg, cno_used_max, cno_all_avg, cno_all_max
    )

    row = [
        timestamp_local, timestamp_gps, f"{lat:.12f}", f"{lon:.12f}",
        f"{hae:.5f}" if hae is not None else "",
        f"{alt_msl:.5f}" if alt_msl is not None else "",
        fixq, fix_tx, nums, hdop, vdop, pdop,
        age, refid,
        mode, spd_kn, crs,
        cno_all_avg, cno_all_max, state.get("ntrip_mode"),
    ]
    csv_writer.writerow(row)

def read_and_log(ser, stop_evt, latest_gga, csv_writer, rtcm_fp, state, status_path, indicator: bool):
    buf = b""
    rtcm_buf = bytearray()
    rtcm_counts = Counter()
    MAXBUF = 262_144
    last_indicator_len = 0
    last_status_t = 0.0

    def indicator_print(text: str):
        nonlocal last_indicator_len
        if not indicator:
            return
        pad = " " * max(0, last_indicator_len - len(text))
        print("\r" + text + pad, end="", flush=True)
        last_indicator_len = len(text)

    def maybe_status_write(now_t: float):
        nonlocal last_status_t
        if not status_path:
            return
        if now_t - last_status_t < 0.2:
            return
        last_status_t = now_t
        atomic_write_json(status_path, {
            "ts_local": datetime.now().isoformat(timespec="milliseconds"),
            "fix_quality": state.get("fix_quality"),
            "num_sats": state.get("num_sats"),
            "lat": state.get("last_lat"),
            "lon": state.get("last_lon"),
            "hae_m": state.get("last_hae"),
            "alt_msl_m": state.get("last_alt_msl"),
            "age_diff": state.get("age_diff"),
            "hdop": state.get("hdop"),
            "pdop": state.get("pdop"),
            "vdop": state.get("vdop"),
            "cno_used_avg": state.get("cno_used_avg"),
            "cno_used_max": state.get("cno_used_max"),
            "cno_all_avg": state.get("cno_all_avg"),
            "cno_all_max": state.get("cno_all_max"),
            "sigma_lat": state.get("sigma_lat"),
            "sigma_lon": state.get("sigma_lon"),
            "sigma_alt": state.get("sigma_alt"),
            "csv_name": state.get("csv_name"),
            "running": True,
        })

    while not stop_evt.is_set():
        try:
            chunk = ser.read(4096)
            if chunk:
                buf += chunk
                # also parse raw RTCM3 frames from the same serial stream
                rtcm_buf.extend(chunk)
                frames = parse_rtcm_frames(rtcm_buf)
                for frame, msg_num in frames:
                    rtcm_fp.write(frame)
                    rtcm_counts[msg_num] += 1

            if len(buf) > MAXBUF:
                buf = buf[-65_536:]

            while True:
                start = buf.find(b'$')
                if start == -1:
                    break
                if start > 0:
                    buf = buf[start:]

                end_n = buf.find(b'\n', 1)
                end_r = buf.find(b'\r', 1)
                if end_n == -1 and end_r == -1:
                    break
                if end_n == -1:
                    end = end_r
                elif end_r == -1:
                    end = end_n
                else:
                    end = end_n if end_n < end_r else end_r

                raw = buf[:end]
                consume = end + 1
                if end + 1 < len(buf) and buf[end:end+2] == b'\r\n':
                    consume = end + 2
                buf = buf[consume:]
                line = raw

                if len(line) >= 6 and line[3:6] == b"RMC":
                    f = line.split(b',')
                    dt = parse_rmc_datetime_utc(f)
                    if dt:
                        state["rmc_date"] = dt.date()

                    try: state["speed_kn"] = float(f[7]) if len(f) > 7 and f[7] else None
                    except Exception: state["speed_kn"] = None
                    try: state["course_deg"] = float(f[8]) if len(f) > 8 and f[8] else None
                    except Exception: state["course_deg"] = None

                    if len(f) > 12 and f[-1]:
                        try:
                            state["nav_mode"] = f[-1].decode('ascii','ignore').split('*',1)[0][:1]
                        except Exception:
                            state["nav_mode"] = None

                    lat = nmea_dm_to_deg(f[3], f[4]) if len(f) > 5 else None
                    lon = nmea_dm_to_deg(f[5], f[6]) if len(f) > 7 else None
                    if lat is not None and lon is not None and not state["saw_gga"]:
                        dt_utc = dt or combine_time_with_date(f[1] if len(f) > 1 else b"", datetime.now(timezone.utc).date())
                        write_csv_row(csv_writer, lat, lon, None, None, dt_utc, state, status_path)
                    continue

                if len(line) >= 6 and line[3:6] == b"GGA":
                    state["saw_gga"] = True
                    latest_gga["bytes"] = line + b"\r\n"
                    f = line.split(b',')

                    utc_time = f[1] if len(f) > 1 else b""
                    lat = nmea_dm_to_deg(f[2], f[3]) if len(f) > 5 else None
                    lon = nmea_dm_to_deg(f[4], f[5]) if len(f) > 7 else None

                    try: state["fix_quality"] = int(f[6]) if len(f) > 6 and f[6] else None
                    except Exception: state["fix_quality"] = None
                    try: state["num_sats"] = int(f[7]) if len(f) > 7 and f[7] else None
                    except Exception: state["num_sats"] = None
                    try:
                        state["hdop"] = float(f[8]) if len(f) > 8 and f[8] else state.get("hdop")
                    except Exception:
                        pass
                    try: state["age_diff"] = float(f[13]) if len(f) > 13 and f[13] else None
                    except Exception: state["age_diff"] = None
                    try: state["ref_id"] = f[14].decode('ascii','ignore').split('*',1)[0] if len(f) > 14 and f[14] else None
                    except Exception: state["ref_id"] = None

                    alt_msl = None
                    geoid = None
                    try:
                        if len(f) > 9 and f[9]:
                            alt_msl = float(f[9].decode('ascii','ignore'))
                    except Exception:
                        pass
                    try:
                        if len(f) > 11 and f[11]:
                            geoid = float(f[11].decode('ascii','ignore'))
                    except Exception:
                        pass
                    hae = (alt_msl + geoid) if (alt_msl is not None and geoid is not None) else None

                    dt_date = state["rmc_date"] or datetime.now(timezone.utc).date()
                    dt_utc = combine_time_with_date(utc_time, dt_date)

                    if lat is not None and lon is not None:
                        state["last_lat"] = lat
                        state["last_lon"] = lon
                        state["last_alt_msl"] = alt_msl
                        state["last_hae"] = hae

                        alt_print = hae if hae is not None else alt_msl
                        tloc = datetime.now().isoformat(timespec="seconds")
                        indicator_print(
                            f"{tloc} | Lat={lat:.7f} Lon={lon:.7f} Alt={alt_print:.2f}m FixQ={state.get('fix_quality')} Sats={state.get('num_sats')}"
                        )

                    write_csv_row(csv_writer, lat, lon, hae, alt_msl, dt_utc, state, status_path)
                    maybe_status_write(time.time())
                    continue

                if len(line) >= 6 and line[3:6] == b"GSA":
                    f = line.split(b',')
                    used = set()
                    for i in range(3, 15):
                        if len(f) > i and f[i]:
                            used.add(f[i].decode('ascii','ignore'))
                    state["gsa_used_prns"] = used
                    try: state["pdop"] = float(f[15]) if len(f) > 15 and f[15] else None
                    except Exception: state["pdop"] = None
                    try: state["hdop"] = float(f[16]) if len(f) > 16 and f[16] else state.get("hdop")
                    except Exception: pass
                    try:
                        fld = f[17].split(b'*', 1)[0] if len(f) > 17 and f[17] else b''
                        state["vdop"] = float(fld) if fld else None
                    except Exception:
                        state["vdop"] = None
                    continue

                if len(line) >= 6 and line[3:6] == b"GSV":
                    f = line.split(b',')
                    i = 4
                    while i + 3 < len(f):
                        prn = f[i].decode('ascii','ignore') if f[i] else None
                        cno_field = f[i+3].split(b'*', 1)[0] if f[i+3] else b''
                        try:
                            cno = int(cno_field) if cno_field else None
                        except Exception:
                            cno = None
                        if prn and cno is not None:
                            state["gsv_all_cno"][prn] = cno
                        i += 4
                    cno_used_avg, cno_used_max, cno_all_avg, cno_all_max = cn0_summaries(state)
                    state["cno_used_avg"] = cno_used_avg
                    state["cno_used_max"] = cno_used_max
                    state["cno_all_avg"]  = cno_all_avg
                    state["cno_all_max"]  = cno_all_max
                    continue

                if len(line) >= 6 and line[3:6] == b"GST":
                    f = line.split(b',')
                    try: state["sigma_lat"] = float(f[6]) if len(f) > 6 and f[6] else None
                    except Exception: state["sigma_lat"] = None
                    try: state["sigma_lon"] = float(f[7]) if len(f) > 7 and f[7] else None
                    except Exception: state["sigma_lon"] = None
                    try:
                        fld = f[8].split(b'*', 1)[0] if len(f) > 8 and f[8] else b''
                        state["sigma_alt"] = float(fld) if fld else None
                    except Exception:
                        state["sigma_alt"] = None
                    continue

        except OSError:
            break
        except Exception as e:
            utc_log(f"serial read error: {e}")
            time.sleep(0.3)

    if status_path:
        atomic_write_json(status_path, {
            "ts_local": datetime.now().isoformat(timespec="milliseconds"),
            "running": False,
            "fix_quality": state.get("fix_quality"),
            "num_sats": state.get("num_sats"),
            "lat": state.get("last_lat"),
            "lon": state.get("last_lon"),
            "hae_m": state.get("last_hae"),
            "csv_name": state.get("csv_name"),
        })
    if indicator:
        print()

# IBGE-only acquisition entry point.
def main():
    ap = argparse.ArgumentParser(description="LC29H NTRIP rover + CSV (HAE)")
    ap.add_argument("--device", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=115200)

    # The correction source is exclusively the official IBGE caster.
    ap.add_argument("--mount", "--ibge-mount", default="RJNI0",
                    help="IBGE station mountpoint (default: RJNI0)")
    ap.add_argument("--outdir", default=".", help="directory for CSV and rover RTCM logs")
    ap.add_argument("--status", default="", help="optional status JSON path")
    ap.add_argument("--indicator", action="store_true", help="show live GNSS status")
    ap.add_argument("--retry-wait", type=float, default=10.0,
                    help="seconds between IBGE reconnection attempts")
    args = ap.parse_args()
    if args.baud <= 0 or args.retry_wait <= 0:
        ap.error("--baud and --retry-wait must be positive")
    args.mount = args.mount.strip().strip("/")
    if not args.mount or any(c.isspace() for c in args.mount):
        ap.error("--mount must be a nonempty IBGE mountpoint without spaces")
    args.caster = "gps-ntrip.ibge.gov.br"
    args.port = 2101
    try:
        args.user, args.password = validate_ibge_credentials()
    except ValueError as exc:
        ap.error(str(exc))
    utc_log(f"Correction source: IBGE | {args.caster}:{args.port}/{args.mount}")

    outdir = Path(args.outdir).expanduser().resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    status_path = Path(args.status).expanduser().resolve() if args.status else None

    rtcm_name = datetime.now().strftime("rover_%Y%m%d_%H%M%S.rtcm3")
    rtcm_path = outdir / rtcm_name
    rtcm_fp = open(rtcm_path, "wb", buffering=0)

    utc_log(f"Opening serial {args.device} @ {args.baud}…")
    ser = serial.Serial(args.device, args.baud, timeout=0.2)

    csv_name = datetime.now().strftime("rtkoutput_%Y%m%d_%H%M%S_svc=ibge.csv")
    csv_path = outdir / csv_name
    csv_fp = open(csv_path, "w", newline="")
    csv_writer = csv.writer(csv_fp)
    csv_writer.writerow([
        "Timestamp_Local", "Timestamp_GPS", "Latitude", "Longitude",
        "Altitude", "Altitude_MSL",
        "FixQuality", "FixText", "NumSats", "HDOP", "VDOP", "PDOP",
        "AgeDiff", "RefStationID",
        "NavMode", "Speed_kn", "Course_deg",
        "CNOavgAll", "CNOmaxAll", "NtripMode"
    ])
    csv_fp.flush()

    utc_log(f"Writing CSV to {csv_path.name}")

    global_stop_evt = threading.Event()
    latest_gga = {"bytes": None}
    state = {
        "csv_name": csv_path.name,
        "rmc_date": None,
        "saw_gga": False,

        "fix_quality": None,
        "num_sats": None,
        "hdop": None,
        "age_diff": None,
        "ref_id": None,

        "nav_mode": None,
        "speed_kn": None,
        "course_deg": None,

        "pdop": None,
        "vdop": None,
        "gsa_used_prns": set(),

        "gsv_all_cno": {},
        "sigma_lat": None,
        "sigma_lon": None,
        "sigma_alt": None,

        "last_lat": None,
        "last_lon": None,
        "last_alt_msl": None,
        "last_hae": None,

        "cno_used_avg": None,
        "cno_used_max": None,
        "cno_all_avg": None,
        "cno_all_max": None,

        "ntrip_mode": "STANDALONE",
    }

    t_read = threading.Thread(
        target=read_and_log,
        args=(ser, global_stop_evt, latest_gga, csv_writer, rtcm_fp, state, status_path, args.indicator),
        daemon=True
    )
    t_read.start()

    utc_log("Running GNSS logger. NTRIP is optional; standalone mode is allowed.")

    sock = None

    try:
        while not global_stop_evt.is_set():
            # Try to connect only if no active NTRIP socket
            if sock is None:
                sock = connect_with_retry_once(args, state)

                if sock is not None:
                    link_down_evt = threading.Event()

                    t_rtcm = threading.Thread(
                        target=rtcm_downlink,
                        args=(sock, ser, link_down_evt, global_stop_evt, state),
                        daemon=True
                    )
                    t_gga = threading.Thread(
                        target=gga_uplink,
                        args=(sock, latest_gga, link_down_evt, global_stop_evt, state, 1.0),
                        daemon=True
                    )
                    t_rtcm.start()
                    t_gga.start()

            # If no socket, remain standalone and wait before retry
            if sock is None:
                time.sleep(args.retry_wait)
                csv_fp.flush()
                continue

            # If socket exists, monitor until it dies
            if link_down_evt.is_set():
                try:
                    sock.close()
                except Exception:
                    pass
                sock = None
                time.sleep(args.retry_wait)
                continue

            time.sleep(0.5)
            csv_fp.flush()

    except KeyboardInterrupt:
        utc_log("Stopping…")
        global_stop_evt.set()

    finally:
        global_stop_evt.set()
        time.sleep(0.2)
        try:
            if sock is not None:
                sock.close()
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass
        try:
            csv_fp.close()
        except Exception:
            pass
        try:
            rtcm_fp.close()
        except Exception:
            pass
        utc_log("Bye.")

if __name__ == "__main__":
    main()
