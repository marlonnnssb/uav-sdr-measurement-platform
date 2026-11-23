# GNSS Subsystem — Configuration and Execution

This guide covers LC29H receiver configuration, reception of IBGE NTRIP corrections, and logging of positioning data and rover RTCM observations.

## Prerequisites

Enable the Raspberry Pi UART interface, disable the serial login console, and ensure your user has permission to access the serial device. The scripts use `/dev/ttyAMA0` at 115200 baud by default.

An internet connection and valid IBGE NTRIP credentials are required to receive corrections.

Run the following commands from the `gnss/` directory. From the repository's main directory, enter:

```bash
cd gnss
```

## 1. Install the Python dependency

```bash
python3 -m pip install pyserial
```

## 2. Configure the receiver as a rover

This command applies the rover settings, enables positioning messages and RTCM observation output, saves the configuration, and requests a cold restart.

```bash
python3 lc29h_apply_config.py --profile rover --save --cold
```

Check the generated configuration log to confirm that the receiver accepted the commands. Available features depend on the receiver variant and firmware.

## 3. Configure NMEA output resolution

Set latitude and longitude to eight decimal places in NMEA minutes and altitude to three decimal places.

```bash
python3 lc29h_apply_config.py --profile precision --pos-dp 8 --alt-dp 3 --save
```

These settings control output resolution, not positioning accuracy.

## 4. Enter your IBGE credentials

Edit these variables near the top of your local copy of `ntrip_rover_ibge.py`:

```python
IBGE_USERNAME = "YOUR_IBGE_USERNAME"
IBGE_PASSWORD = "YOUR_IBGE_PASSWORD"
```

Replace both placeholders with your credentials. The script checks that they have been changed before starting.

**Do not commit your edited credentials to GitHub.**

## 5. Start GNSS acquisition

```bash
python3 ntrip_rover_ibge.py --mount RJNI0 --outdir logs --indicator
```

Replace `RJNI0` with the desired IBGE station mountpoint.

Because these commands run from `gnss/`, the output directory is `gnss/logs/`, relative to the repository's main directory.

The script saves:

- **CSV files:** positioning data, solution quality, and satellite information.
- **`.rtcm3` files:** raw RTCM observations output by the rover, rather than the incoming IBGE correction stream.

RTCM observation logging requires the receiver to accept the corresponding output configuration.

## 6. Check the connection and positioning status

A successful NTRIP connection displays:

```text
NTRIP connected (200)
```

This confirms that the correction service accepted the connection. It does not indicate that an RTK fixed solution has been obtained.

The `FixQ` field reports positioning status:

| FixQ | Positioning status |
|---|---|
| 1 | Standalone GNSS |
| 2 | Differential GNSS |
| 4 | RTK fixed |
| 5 | RTK float |

If NTRIP becomes unavailable, the script continues recording available GNSS positioning data and attempts to reconnect.

Press **Ctrl+C** to stop acquisition.

**Run the configuration scripts and GNSS logger separately:** they access the same UART port. These scripts configure serial communication in the application; they do not enable the Raspberry Pi UART interface or disable its serial login console.
