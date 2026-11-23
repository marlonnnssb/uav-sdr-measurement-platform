# RF Acquisition Subsystem

GNU Radio flowgraph and Python scripts for narrowband RF acquisition.

## Acquisition flowgraph

The flowgraph acquires IQ samples from the LimeSDR Mini, computes
the FFT, averages the spectral power, and applies a calibration
offset. The resulting spectrum is displayed and selected frequency
bins around the carrier are saved to CSV.

![GNU Radio RF acquisition flowgraph](/docs/gnuradio_flowchart.png)
