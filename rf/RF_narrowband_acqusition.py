#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: RF acquisition
# Author: Marlon Borges
# GNU Radio version: 3.10.12.0

from PyQt5 import Qt
from gnuradio import qtgui
from PyQt5 import QtCore
from gnuradio import blocks
from gnuradio import fft
from gnuradio.fft import window
from gnuradio import gr
from gnuradio.filter import firdes
import sys
import signal
from PyQt5 import Qt
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import soapy
import RF_narrowband_acqusition_epy_block_0 as epy_block_0  # embedded python block
import RF_narrowband_acqusition_epy_block_0_0 as epy_block_0_0  # embedded python block
import RF_narrowband_acqusition_epy_block_1 as epy_block_1  # embedded python block
import numpy as np
import sip
import threading



class RF_narrowband_acqusition(gr.top_block, Qt.QWidget):

    def __init__(self, outdir='logs/rf'):
        gr.top_block.__init__(self, "RF acquisition", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("RF acquisition")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except BaseException as exc:
            print(f"Qt GUI: Could not set Icon: {str(exc)}", file=sys.stderr)
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("gnuradio/flowgraphs", "RF_narrowband_acqusition")

        try:
            geometry = self.settings.value("geometry")
            if geometry:
                self.restoreGeometry(geometry)
        except BaseException as exc:
            print(f"Qt GUI: Could not restore geometry: {str(exc)}", file=sys.stderr)
        self.flowgraph_started = threading.Event()

        ##################################################
        # Parameters
        ##################################################
        self.outdir = outdir

        ##################################################
        # Variables
        ##################################################
        self.f_cw = f_cw = 3.50e9
        self.NFFT = NFFT = 1024
        self.search_BW = search_BW = 5e3
        self.samp_rate = samp_rate = 1000000
        self.norm_factor = norm_factor = 1.0/(NFFT*NFFT)
        self.gain = gain = 10
        self.freq = freq = f_cw -200e3
        self.avg_len = avg_len = 1
        self.RSSI_Offset = RSSI_Offset = 8

        ##################################################
        # Blocks
        ##################################################

        self._samp_rate_range = qtgui.Range(1000, 20000000, 1000, 1000000, 200)
        self._samp_rate_win = qtgui.RangeWidget(self._samp_rate_range, self.set_samp_rate, "sample rate", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._samp_rate_win)
        self._gain_range = qtgui.Range(0, 61, 1, 10, 200)
        self._gain_win = qtgui.RangeWidget(self._gain_range, self.set_gain, "'gain'", "counter_slider", int, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._gain_win)
        self._freq_range = qtgui.Range(f_cw - 1e9, 3.5e9, 1e3, f_cw -200e3, 200)
        self._freq_win = qtgui.RangeWidget(self._freq_range, self.set_freq, "'freq'", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._freq_win)
        self._avg_len_range = qtgui.Range(1, 1000, 10, 1, 200)
        self._avg_len_win = qtgui.RangeWidget(self._avg_len_range, self.set_avg_len, "'avg_len'", "counter_slider", int, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._avg_len_win)
        self._RSSI_Offset_range = qtgui.Range(-50, 50, 1, 8, 200)
        self._RSSI_Offset_win = qtgui.RangeWidget(self._RSSI_Offset_range, self.set_RSSI_Offset, "Offset", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._RSSI_Offset_win)
        self.soapy_limesdr_source_0 = None
        dev = 'driver=lime'
        stream_args = ''
        tune_args = ['']
        settings = ['']

        self.soapy_limesdr_source_0 = soapy.source(dev, "fc32", 1, '',
                                  stream_args, tune_args, settings)
        self.soapy_limesdr_source_0.set_sample_rate(0, samp_rate)
        self.soapy_limesdr_source_0.set_bandwidth(0, samp_rate)
        self.soapy_limesdr_source_0.set_frequency(0, freq)
        self.soapy_limesdr_source_0.set_frequency_correction(0, 0)
        self.soapy_limesdr_source_0.set_gain(0, min(max(gain, -12.0), 61.0))
        self.qtgui_vector_sink_f_0 = qtgui.vector_sink_f(
            NFFT,
            ((freq - samp_rate/2) / 1e6),
            ((samp_rate / NFFT) / 1e6),
            "x-Axis",
            "y-Axis",
            "",
            1, # Number of inputs
            None # parent
        )
        self.qtgui_vector_sink_f_0.set_update_time(0.10)
        self.qtgui_vector_sink_f_0.set_y_axis((-140), 10)
        self.qtgui_vector_sink_f_0.enable_autoscale(False)
        self.qtgui_vector_sink_f_0.enable_grid(True)
        self.qtgui_vector_sink_f_0.set_x_axis_units("")
        self.qtgui_vector_sink_f_0.set_y_axis_units("")
        self.qtgui_vector_sink_f_0.set_ref_level(0)


        labels = ['', '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
            "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_vector_sink_f_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_vector_sink_f_0.set_line_label(i, labels[i])
            self.qtgui_vector_sink_f_0.set_line_width(i, widths[i])
            self.qtgui_vector_sink_f_0.set_line_color(i, colors[i])
            self.qtgui_vector_sink_f_0.set_line_alpha(i, alphas[i])

        self._qtgui_vector_sink_f_0_win = sip.wrapinstance(self.qtgui_vector_sink_f_0.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_vector_sink_f_0_win)
        self.qtgui_number_sink_0 = qtgui.number_sink(
            gr.sizeof_float,
            0,
            qtgui.NUM_GRAPH_HORIZ,
            2,
            None # parent
        )
        self.qtgui_number_sink_0.set_update_time(0.10)
        self.qtgui_number_sink_0.set_title("")

        labels = ['peak (dB)', 'peak (freq)', '', '', '',
            '', '', '', '', '']
        units = ['', '', '', '', '',
            '', '', '', '', '']
        colors = [("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"),
            ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black")]
        factor = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]

        for i in range(2):
            self.qtgui_number_sink_0.set_min(i, -1)
            self.qtgui_number_sink_0.set_max(i, 1)
            self.qtgui_number_sink_0.set_color(i, colors[i][0], colors[i][1])
            if len(labels[i]) == 0:
                self.qtgui_number_sink_0.set_label(i, "Data {0}".format(i))
            else:
                self.qtgui_number_sink_0.set_label(i, labels[i])
            self.qtgui_number_sink_0.set_unit(i, units[i])
            self.qtgui_number_sink_0.set_factor(i, factor[i])

        self.qtgui_number_sink_0.enable_autoscale(False)
        self._qtgui_number_sink_0_win = sip.wrapinstance(self.qtgui_number_sink_0.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_number_sink_0_win)
        self.fft_vxx_0 = fft.fft_vcc(NFFT, True, window.blackmanharris(NFFT), True, 1)
        self.epy_block_1 = epy_block_1.blk(nfft=NFFT, avg_len=avg_len)
        self.epy_block_0_0 = epy_block_0_0.blk(fs=samp_rate, nfft=NFFT, fc=freq, fcw=f_cw, search_bw=search_BW, shifted=True)
        self.epy_block_0 = epy_block_0.blk(csv_path=outdir, fs=samp_rate, nfft=NFFT, fc=freq, shifted=True, freq_unit="mhz", log_every_n=4, gain_db=gain, rssi_offset_db=RSSI_Offset, save_center_hz=f_cw, save_bw_hz=5000)
        self.blocks_stream_to_vector_0 = blocks.stream_to_vector(gr.sizeof_gr_complex*1, NFFT)
        self.blocks_nlog10_ff_0 = blocks.nlog10_ff(10, NFFT, 0)
        self.blocks_multiply_const_vxx_0 = blocks.multiply_const_vff(norm_factor * np.ones(NFFT))
        self.blocks_complex_to_mag_squared_0 = blocks.complex_to_mag_squared(NFFT)
        self.blocks_add_const_vxx_0 = blocks.add_const_vff((RSSI_Offset,)*NFFT)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.blocks_add_const_vxx_0, 0), (self.epy_block_0, 0))
        self.connect((self.blocks_add_const_vxx_0, 0), (self.epy_block_0_0, 0))
        self.connect((self.blocks_add_const_vxx_0, 0), (self.qtgui_vector_sink_f_0, 0))
        self.connect((self.blocks_complex_to_mag_squared_0, 0), (self.blocks_multiply_const_vxx_0, 0))
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.epy_block_1, 0))
        self.connect((self.blocks_nlog10_ff_0, 0), (self.blocks_add_const_vxx_0, 0))
        self.connect((self.blocks_stream_to_vector_0, 0), (self.fft_vxx_0, 0))
        self.connect((self.epy_block_0_0, 1), (self.qtgui_number_sink_0, 1))
        self.connect((self.epy_block_0_0, 0), (self.qtgui_number_sink_0, 0))
        self.connect((self.epy_block_1, 0), (self.blocks_nlog10_ff_0, 0))
        self.connect((self.fft_vxx_0, 0), (self.blocks_complex_to_mag_squared_0, 0))
        self.connect((self.soapy_limesdr_source_0, 0), (self.blocks_stream_to_vector_0, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("gnuradio/flowgraphs", "RF_narrowband_acqusition")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_outdir(self):
        return self.outdir

    def set_outdir(self, outdir):
        self.outdir = outdir
        self.epy_block_0.csv_path = self.outdir

    def get_f_cw(self):
        return self.f_cw

    def set_f_cw(self, f_cw):
        self.f_cw = f_cw
        self.set_freq(self.f_cw -200e3)
        self.epy_block_0.save_center_hz = self.f_cw
        self.epy_block_0_0.fcw = self.f_cw

    def get_NFFT(self):
        return self.NFFT

    def set_NFFT(self, NFFT):
        self.NFFT = NFFT
        self.set_norm_factor(1.0/(self.NFFT*self.NFFT))
        self.blocks_add_const_vxx_0.set_k((self.RSSI_Offset,)*self.NFFT)
        self.blocks_multiply_const_vxx_0.set_k(self.norm_factor * np.ones(self.NFFT))
        self.epy_block_0.nfft = self.NFFT
        self.epy_block_0_0.nfft = self.NFFT
        self.epy_block_1.nfft = self.NFFT
        self.fft_vxx_0.set_window(window.blackmanharris(self.NFFT))
        self.qtgui_vector_sink_f_0.set_x_axis(((self.freq - self.samp_rate/2) / 1e6), ((self.samp_rate / self.NFFT) / 1e6))

    def get_search_BW(self):
        return self.search_BW

    def set_search_BW(self, search_BW):
        self.search_BW = search_BW
        self.epy_block_0_0.search_bw = self.search_BW

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.epy_block_0.fs = self.samp_rate
        self.epy_block_0_0.fs = self.samp_rate
        self.qtgui_vector_sink_f_0.set_x_axis(((self.freq - self.samp_rate/2) / 1e6), ((self.samp_rate / self.NFFT) / 1e6))
        self.soapy_limesdr_source_0.set_sample_rate(0, self.samp_rate)
        self.soapy_limesdr_source_0.set_bandwidth(0, self.samp_rate)

    def get_norm_factor(self):
        return self.norm_factor

    def set_norm_factor(self, norm_factor):
        self.norm_factor = norm_factor
        self.blocks_multiply_const_vxx_0.set_k(self.norm_factor * np.ones(self.NFFT))

    def get_gain(self):
        return self.gain

    def set_gain(self, gain):
        self.gain = gain
        self.epy_block_0.gain_db = self.gain
        self.soapy_limesdr_source_0.set_gain(0, min(max(self.gain, -12.0), 61.0))

    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq
        self.epy_block_0.fc = self.freq
        self.epy_block_0_0.fc = self.freq
        self.qtgui_vector_sink_f_0.set_x_axis(((self.freq - self.samp_rate/2) / 1e6), ((self.samp_rate / self.NFFT) / 1e6))
        self.soapy_limesdr_source_0.set_frequency(0, self.freq)

    def get_avg_len(self):
        return self.avg_len

    def set_avg_len(self, avg_len):
        self.avg_len = avg_len
        self.epy_block_1.avg_len = self.avg_len

    def get_RSSI_Offset(self):
        return self.RSSI_Offset

    def set_RSSI_Offset(self, RSSI_Offset):
        self.RSSI_Offset = RSSI_Offset
        self.blocks_add_const_vxx_0.set_k((self.RSSI_Offset,)*self.NFFT)
        self.epy_block_0.rssi_offset_db = self.RSSI_Offset



def argument_parser():
    parser = ArgumentParser()
    parser.add_argument(
        "--outdir", dest="outdir", type=str, default='logs/rf',
        help="Set Output dir [default=%(default)r]")
    return parser


def main(top_block_cls=RF_narrowband_acqusition, options=None):
    if options is None:
        options = argument_parser().parse_args()

    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls(outdir=options.outdir)

    tb.start()
    tb.flowgraph_started.set()

    tb.show()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    qapp.exec_()

if __name__ == '__main__':
    main()
