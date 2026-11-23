# UAV–SDR Measurement Platform

Acquisition scripts and hardware documentation for a lightweight,
low-cost Raspberry Pi-based platform integrating a LimeSDR Mini,
an LC29H GNSS receiver, and a BNO055 IMU.

The repository provides independent scripts for configuring and
acquiring data from each subsystem. Before execution, assemble the
hardware according to the wiring diagram and follow the dependencies,
configuration, and calibration instructions in each subsystem's README.

## Hardware used

The platform was developed using the following components:

| Component | Model or specification | Function |
|---|---|---|
| Embedded computer | Raspberry Pi 5 | Runs the acquisition scripts and stores measurement data |
| SDR receiver | LimeSDR Mini v2.4 | Acquires RF signals through a USB 2.0 connection |
| GNSS receiver | Waveshare LC29H(DA) GNSS HAT | Provides positioning, RTK capability, and rover observations through UART |
| GNSS antenna | External antenna supplied with the LC29H HAT | Receives GNSS signals |
| Orientation sensor | Adafruit BNO055 breakout board | Provides orientation and inertial measurements through I²C |
| RF receiving antenna | Directional patch antenna for 3.5 GHz | Receives the signal under measurement |
| Internet connection | ZTE MF920U 4G modem/router | Provides Wi-Fi internet access for receiving IBGE NTRIP corrections |
| Power supply | Power Bank i2GO 5000 mAh | Powers the acquisition platform |

The integrated payload had a total mass of approximately **442 g**.
The power bank supplied the acquisition system for more than **120 minutes**
during testing.

The supplied RF configuration targets narrowband measurements at **3.5 GHz**.
Using different hardware or frequency settings may require changes to the
drivers, acquisition parameters, antenna selection, and calibration.

Refer to the [wiring diagram](docs/platform_wiring.png) and each subsystem's
README before assembling and operating the platform.

## Hardware connections

The diagram below shows the platform wiring, including the LC29H
GNSS HAT, BNO055 IMU, LimeSDR Mini, and modem power connections.

![UAV–SDR platform wiring diagram](docs/platform_wiring.png)

The LC29H HAT mounts directly onto the Raspberry Pi's 40-pin header
and communicates through UART. The BNO055 uses I²C, while the
LimeSDR Mini connects through USB 2.0.

To reduce the payload weight, the 4G modem/router was modified by removing its
internal battery and enclosure. The adapted device was powered directly from 
the Raspberry Pi's 5 V supply and ground pins, as shown in the wiring diagram.

## Subsystems

- [RF acquisition](rf/README.md): GNU Radio flowgraph, spectrum
  display, and narrowband spectral logging.
- [GNSS positioning and acquisition](gnss/README.md): receiver
  configuration, IBGE NTRIP corrections, positioning data,
  and rover RTCM observation logging.
- [IMU calibration and acquisition](imu/README.md): sensor calibration,
  orientation estimation, and inertial measurement logging.

> **Before running the platform:** Read each subsystem's documentation
> and complete its hardware configuration and calibration procedures.

## Suggested acquisition sequence

The subsystems can run independently in separate terminals.
Users may also develop their own orchestrator to automate startup,
monitor execution, and organize output files.

An orchestrator is not included in this repository. The suggested
startup order is:

1. **Start GNSS acquisition.** Confirm that positioning data are being
   recorded and wait for the solution quality required by the experiment.
   For measurements requiring RTK fixed positioning, wait for `FixQ=4`.
2. **Calibrate the IMU if needed.** Complete calibration before starting
   the IMU logger; these two scripts must run separately.
3. **Start IMU acquisition.** Keep the platform stationary during
   stabilization and wait until CSV logging begins.
4. **Start RF acquisition.** Execute the supplied flowgraph in
   GNU Radio Companion and confirm that the RF CSV is being written.
5. **Begin the experiment** once all three subsystems are recording.

## Output files and timing

Each subsystem writes its own measurement files to its configured
output directory. Refer to its README for the file format and settings.

Starting the scripts in sequence does not provide hardware synchronization.
Keep the Raspberry Pi's clock correctly configured and account for
timestamp conventions and acquisition latency when combining the logs.
