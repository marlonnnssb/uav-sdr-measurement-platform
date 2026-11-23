### Hardware connections

The diagram below shows the platform wiring, including the LC29H
GNSS HAT, BNO055 IMU, LimeSDR Mini, and modem power connections.

![UAV–SDR platform wiring diagram](docs/platform_wiring.png)

The LC29H HAT mounts directly onto the Raspberry Pi's 40-pin header
and communicates through UART. The BNO055 uses the I²C interface,
while the LimeSDR Mini connects through USB 2.0.

> **Before running the platform:** Read the documentation for each
> subsystem for its dependencies, hardware configuration, calibration,
> and execution instructions.

- [RF acquisition](rf/README.md)
- [GNSS positioning and acquisition](gnss/README.md)
- [IMU orientation and acquisition](imu/README.md)
