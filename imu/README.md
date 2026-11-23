# IMU Subsystem — Calibration and Acquisition

Python scripts for calibrating the BNO055 and recording orientation
and inertial measurements through the Raspberry Pi's I²C interface.

## 1. Install the dependencies

Before running either script, install and configure **Adafruit Blinka**
and the **Adafruit CircuitPython BNO055 library** using these guides:

- [Installing Blinka on Raspberry Pi](https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/installing-circuitpython-on-raspberry-pi)
- [BNO055 Python installation and wiring](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/python-circuitpython)

The scripts require:

- Python 3
- NumPy
- Adafruit Blinka
- Adafruit CircuitPython BNO055

Using the Python environment configured in the Adafruit guide, install:

```bash
python3 -m pip install numpy adafruit-blinka adafruit-circuitpython-bno055
```

If using a virtual environment, activate it before installing dependencies
and whenever running the scripts.

## 2. Enable I²C and check the connection

Run:

```bash
sudo raspi-config
```

Select **Interface Options → I2C → Enable** and reboot if prompted.

Connect the sensor according to the
[platform wiring diagram](../docs/platform_wiring.png).

Install the I²C diagnostic tools and check detection:

```bash
sudo apt install i2c-tools
i2cdetect -y 1
```

The scripts use the BNO055's default I²C address, **0x28**.
If your sensor uses another address, update the sensor initialization
in both scripts.

Disabling Bluetooth is not required by these I²C scripts.

## 3. Open the IMU directory

From the repository's main directory:

```bash
cd imu
```

Keep these files together:

- `calibrate_BNO055.py`
- `IMU_logger.py`

The calibration script saves `bno055_offsets.json` beside the scripts,
and the logger loads it from that location.

## 4. Calibrate the sensor

Mount the IMU in its final position and keep the surrounding equipment
arranged as it will be during measurements. Keep the UAV motors stopped.

Run:

```bash
python3 calibrate_BNO055.py
```

Full calibration is enabled by default. Follow the terminal instructions:

1. **Magnetometer:** slowly move the complete assembly through varied
   orientations, including figure-eight movements.
2. **Accelerometer:** place the assembly in six stable orientations.
3. **Gyroscope:** keep the assembly stationary.

The script waits for each sensor's calibration level to reach **3**
and saves the offsets to:

```text
bno055_offsets.json
```

These offsets belong to your sensor and installation. Recalibrate after
changing the mounting or nearby magnetic environment.

The supplied scripts contain no preset sensor-specific offsets.

## 5. Start IMU acquisition

After calibration:

```bash
python3 IMU_logger.py --outdir logs
```

Keep the assembly stationary during the initial stabilization phase.
The logger establishes a starting heading reference before recording.

The logger requires a valid `bno055_offsets.json`. If the file is missing
or invalid, it stops and asks you to run the calibration script.

Press **Ctrl+C** to stop recording.

Run calibration and acquisition separately so both scripts do not access
the sensor simultaneously.

## 6. Recorded data

When launched from `imu/`, the command above saves CSV files in
`imu/logs/`, with names such as:

```text
sensor_log_YYYYMMDD_HHMMSS.csv
```

The CSV contains:

| Measurements | Units or convention |
|---|---|
| Heading, roll, pitch | Degrees; pitch sign is inverted by the logger |
| Relative yaw | Heading difference from the initial reference, wrapped to −180° to 180° |
| Quaternion | Components ordered as w, x, y, z |
| Acceleration | m/s², including gravity |
| Angular velocity | rad/s |
| Magnetic field | µT |
| Linear acceleration | m/s², excluding gravity |
| Gravity vector | m/s² |
| Calibration status | Levels from 0 to 3 for system, gyroscope, accelerometer, and magnetometer |
| Sensor temperature | °C |

The default operating mode is **NDOF**, which uses the magnetometer
for heading estimation. No magnetic-declination correction is applied.

The logger targets **80 samples/s**. Actual recording rate depends
on I²C communication and system load; polling frequency does not guarantee
that every sensor channel provides a new measurement at each sample.

Timestamps use the Raspberry Pi's local system clock.

## Troubleshooting

- **Sensor not detected:** check power, ground, SDA/SCL wiring, I²C
  configuration, and sensor address.
- **Import errors:** confirm that the dependencies are installed in
  the active Python environment.
- **Calibration does not finish:** follow the requested movements and
  check for nearby magnetic interference.
- **Logger remains in stabilization:** keep the assembly still and
  check whether heading readings are stable.
- **Intermittent I²C errors:** consult Adafruit's
  [I²C clock-stretching guidance](https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/i2c-clock-stretching).
