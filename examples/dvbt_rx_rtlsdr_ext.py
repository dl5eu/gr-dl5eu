#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: DVB-T RX with low bandwidth on 70cm
# Author: Ralf DL5EU
# Description: DVB-T reception with low bandwidth on 70cm using an RTL-SDR stick
# GNU Radio version: 3.10.12.0

from PyQt5 import Qt
from gnuradio import qtgui
from PyQt5 import QtCore
from PyQt5.QtCore import QObject, pyqtSlot
from gnuradio import blocks
from gnuradio import dl5eu
from gnuradio import dtv
from gnuradio import filter
from gnuradio.filter import firdes
from gnuradio import gr
from gnuradio.fft import window
import sys
import signal
from PyQt5 import Qt
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import iio
from gnuradio import network
import configparser
import dvbt_rx_rtlsdr_ext_bandwidth_decim_factor as bandwidth_decim_factor  # embedded python module
import sip
import threading



class dvbt_rx_rtlsdr_ext(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "DVB-T RX with low bandwidth on 70cm", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("DVB-T RX with low bandwidth on 70cm")
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

        self.settings = Qt.QSettings("gnuradio/flowgraphs", "dvbt_rx_rtlsdr_ext")

        try:
            geometry = self.settings.value("geometry")
            if geometry:
                self.restoreGeometry(geometry)
        except BaseException as exc:
            print(f"Qt GUI: Could not restore geometry: {str(exc)}", file=sys.stderr)
        self.flowgraph_started = threading.Event()

        ##################################################
        # Variables
        ##################################################
        self._conf_decim_select_config = configparser.ConfigParser()
        self._conf_decim_select_config.read('config.ini')
        try: conf_decim_select = self._conf_decim_select_config.getint('setup', 'decim_select')
        except: conf_decim_select = 1
        self.conf_decim_select = conf_decim_select
        self.decim_select = decim_select = conf_decim_select
        self._conf_rf_gain_db_config = configparser.ConfigParser()
        self._conf_rf_gain_db_config.read('config.ini')
        try: conf_rf_gain_db = self._conf_rf_gain_db_config.getint('setup', 'rf_gain_db')
        except: conf_rf_gain_db = 10
        self.conf_rf_gain_db = conf_rf_gain_db
        self._conf_if_gain_db_config = configparser.ConfigParser()
        self._conf_if_gain_db_config.read('config.ini')
        try: conf_if_gain_db = self._conf_if_gain_db_config.getint('setup', 'if_gain_db')
        except: conf_if_gain_db = 10
        self.conf_if_gain_db = conf_if_gain_db
        self._conf_freq_corr_ppm_config = configparser.ConfigParser()
        self._conf_freq_corr_ppm_config.read('config.ini')
        try: conf_freq_corr_ppm = self._conf_freq_corr_ppm_config.getint('setup', 'freq_corr_ppm')
        except: conf_freq_corr_ppm = 0
        self.conf_freq_corr_ppm = conf_freq_corr_ppm
        self._conf_center_freq_config = configparser.ConfigParser()
        self._conf_center_freq_config.read('config.ini')
        try: conf_center_freq = self._conf_center_freq_config.getint('setup', 'center_freq')
        except: conf_center_freq = 437000000
        self.conf_center_freq = conf_center_freq
        self.sync_sample_rate = sync_sample_rate = 2400000 * [20/21, 10/21, 5/21, 160/1009, 5/42, 5/84, 280/8909][decim_select]
        self.squelch_threshold = squelch_threshold = -60
        self.rf_gain_db = rf_gain_db = conf_rf_gain_db
        self.if_gain_db = if_gain_db = conf_if_gain_db
        self.freq_corr_ppm = freq_corr_ppm = conf_freq_corr_ppm
        self.center_freq = center_freq = conf_center_freq

        ##################################################
        # Blocks
        ##################################################

        self._squelch_threshold_range = qtgui.Range(-200, 200, 1, -60, 200)
        self._squelch_threshold_win = qtgui.RangeWidget(self._squelch_threshold_range, self.set_squelch_threshold, "Squelch", "counter", float, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._squelch_threshold_win)
        self._rf_gain_db_range = qtgui.Range(0, 42, 1, conf_rf_gain_db, 0)
        self._rf_gain_db_win = qtgui.RangeWidget(self._rf_gain_db_range, self.set_rf_gain_db, "RF gain", "counter", int, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._rf_gain_db_win, 1, 0, 1, 1)
        for r in range(1, 2):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        # Create the options list
        self._decim_select_options = [0, 1, 2, 3, 4, 5, 6]
        # Create the labels list
        self._decim_select_labels = ['2 MHz', '1 MHz', '500 kHz', '333 kHz', '250 kHz', '125 kHz', '66 kHz']
        # Create the combo box
        self._decim_select_tool_bar = Qt.QToolBar(self)
        self._decim_select_tool_bar.addWidget(Qt.QLabel("Bandwidth" + ": "))
        self._decim_select_combo_box = Qt.QComboBox()
        self._decim_select_tool_bar.addWidget(self._decim_select_combo_box)
        for _label in self._decim_select_labels: self._decim_select_combo_box.addItem(_label)
        self._decim_select_callback = lambda i: Qt.QMetaObject.invokeMethod(self._decim_select_combo_box, "setCurrentIndex", Qt.Q_ARG("int", self._decim_select_options.index(i)))
        self._decim_select_callback(self.decim_select)
        self._decim_select_combo_box.currentIndexChanged.connect(
            lambda i: self.set_decim_select(self._decim_select_options[i]))
        # Create the radio buttons
        self.top_grid_layout.addWidget(self._decim_select_tool_bar, 7, 0, 1, 1)
        for r in range(7, 8):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        self._center_freq_range = qtgui.Range(430000000, 440000000, 100, conf_center_freq, 0)
        self._center_freq_win = qtgui.RangeWidget(self._center_freq_range, self.set_center_freq, "Center freq.", "counter", int, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._center_freq_win, 2, 0, 1, 1)
        for r in range(2, 3):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.samp_error = qtgui.number_sink(
            gr.sizeof_float,
            0,
            qtgui.NUM_GRAPH_HORIZ,
            1,
            None # parent
        )
        self.samp_error.set_update_time(0.25)
        self.samp_error.set_title('')

        labels = ['Samp. error (ppm)', '', '', '', '',
            '', '', '', '', '']
        units = ['', '', '', '', '',
            '', '', '', '', '']
        colors = [("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"),
            ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black")]
        factor = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]

        for i in range(1):
            self.samp_error.set_min(i, -100)
            self.samp_error.set_max(i, 100)
            self.samp_error.set_color(i, colors[i][0], colors[i][1])
            if len(labels[i]) == 0:
                self.samp_error.set_label(i, "Data {0}".format(i))
            else:
                self.samp_error.set_label(i, labels[i])
            self.samp_error.set_unit(i, units[i])
            self.samp_error.set_factor(i, factor[i])

        self.samp_error.enable_autoscale(False)
        self._samp_error_win = sip.wrapinstance(self.samp_error.qwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._samp_error_win, 6, 0, 1, 2)
        for r in range(6, 7):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.rational_resampler_xxx_4 = filter.rational_resampler_ccc(
                interpolation=5,
                decimation=84,
                taps=[],
                fractional_bw=0)
        self.rational_resampler_xxx_3_0 = filter.rational_resampler_ccc(
                interpolation=280,
                decimation=8909,
                taps=[],
                fractional_bw=0)
        self.rational_resampler_xxx_3 = filter.rational_resampler_ccc(
                interpolation=5,
                decimation=42,
                taps=[],
                fractional_bw=0)
        self.rational_resampler_xxx_2_0 = filter.rational_resampler_ccc(
                interpolation=160,
                decimation=1009,
                taps=[],
                fractional_bw=0)
        self.rational_resampler_xxx_2 = filter.rational_resampler_ccc(
                interpolation=5,
                decimation=21,
                taps=[],
                fractional_bw=0)
        self.rational_resampler_xxx_1 = filter.rational_resampler_ccc(
                interpolation=10,
                decimation=21,
                taps=[],
                fractional_bw=0)
        self.rational_resampler_xxx_0 = filter.rational_resampler_ccc(
                interpolation=20,
                decimation=21,
                taps=[],
                fractional_bw=0)
        self.qtgui_const_sink_x_0 = qtgui.const_sink_c(
            512, #size
            'Constellation', #name
            1, #number of inputs
            None # parent
        )
        self.qtgui_const_sink_x_0.set_update_time(0.10)
        self.qtgui_const_sink_x_0.set_y_axis((-1.5), 1.5)
        self.qtgui_const_sink_x_0.set_x_axis((-1.5), 1.5)
        self.qtgui_const_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, "")
        self.qtgui_const_sink_x_0.enable_autoscale(False)
        self.qtgui_const_sink_x_0.enable_grid(True)
        self.qtgui_const_sink_x_0.enable_axis_labels(True)


        labels = [" ", '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "red", "red", "red",
            "red", "red", "red", "red", "red"]
        styles = [0, 0, 0, 0, 0,
            0, 0, 0, 0, 0]
        markers = [0, 0, 0, 0, 0,
            0, 0, 0, 0, 0]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_const_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_const_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_const_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_const_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_const_sink_x_0.set_line_style(i, styles[i])
            self.qtgui_const_sink_x_0.set_line_marker(i, markers[i])
            self.qtgui_const_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_const_sink_x_0_win = sip.wrapinstance(self.qtgui_const_sink_x_0.qwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._qtgui_const_sink_x_0_win, 0, 0, 1, 2)
        for r in range(0, 1):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.network_udp_sink_0 = network.udp_sink(gr.sizeof_char, 1, '230.0.0.1', 2000, 0, 1316, True)
        self.iio_pluto_source_0 = iio.fmcomms2_source_fc32('' if '' else iio.get_pluto_uri(), [True, True], 32768)
        self.iio_pluto_source_0.set_len_tag_key('packet_len')
        self.iio_pluto_source_0.set_frequency(center_freq)
        self.iio_pluto_source_0.set_samplerate(2400000)
        self.iio_pluto_source_0.set_gain_mode(0, 'manual')
        self.iio_pluto_source_0.set_gain(0, rf_gain_db)
        self.iio_pluto_source_0.set_quadrature(True)
        self.iio_pluto_source_0.set_rfdc(True)
        self.iio_pluto_source_0.set_bbdc(True)
        self.iio_pluto_source_0.set_filter_params('Auto', '', 0, 0)
        self._if_gain_db_range = qtgui.Range(0, 40, 1, conf_if_gain_db, 0)
        self._if_gain_db_win = qtgui.RangeWidget(self._if_gain_db_range, self.set_if_gain_db, "IF gain", "counter", int, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._if_gain_db_win, 1, 1, 1, 1)
        for r in range(1, 2):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(1, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.freq_error = qtgui.number_sink(
            gr.sizeof_float,
            0,
            qtgui.NUM_GRAPH_HORIZ,
            1,
            None # parent
        )
        self.freq_error.set_update_time(0.25)
        self.freq_error.set_title('')

        labels = ['Freq. error (Hz)', '', '', '', '',
            '', '', '', '', '']
        units = ['', '', '', '', '',
            '', '', '', '', '']
        colors = [("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"),
            ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black"), ("black", "black")]
        factor = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]

        for i in range(1):
            self.freq_error.set_min(i, -10000)
            self.freq_error.set_max(i, 10000)
            self.freq_error.set_color(i, colors[i][0], colors[i][1])
            if len(labels[i]) == 0:
                self.freq_error.set_label(i, "Data {0}".format(i))
            else:
                self.freq_error.set_label(i, labels[i])
            self.freq_error.set_unit(i, units[i])
            self.freq_error.set_factor(i, factor[i])

        self.freq_error.enable_autoscale(False)
        self._freq_error_win = sip.wrapinstance(self.freq_error.qwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._freq_error_win, 5, 0, 1, 2)
        for r in range(5, 6):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self._freq_corr_ppm_range = qtgui.Range((-100), 100, 1, conf_freq_corr_ppm, 0)
        self._freq_corr_ppm_win = qtgui.RangeWidget(self._freq_corr_ppm_range, self.set_freq_corr_ppm, "Freq. ppm", "counter", int, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._freq_corr_ppm_win, 2, 1, 1, 1)
        for r in range(2, 3):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(1, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.dtv_dvbt_symbol_inner_interleaver_0 = dtv.dvbt_symbol_inner_interleaver(1512, dtv.T2k, 0)
        self.dtv_dvbt_reed_solomon_dec_0 = dtv.dvbt_reed_solomon_dec(2, 8, 0x11d, 255, 239, 8, 51, 8)
        self.dtv_dvbt_energy_descramble_0 = dtv.dvbt_energy_descramble(8)
        self.dtv_dvbt_demap_0 = dtv.dvbt_demap(1512, dtv.MOD_16QAM, dtv.NH, dtv.T2k, 1)
        self.dtv_dvbt_convolutional_deinterleaver_0 = dtv.dvbt_convolutional_deinterleaver(136, 12, 17)
        self.dtv_dvbt_bit_inner_deinterleaver_0 = dtv.dvbt_bit_inner_deinterleaver(1512, dtv.MOD_16QAM, dtv.NH, dtv.T2k)
        self.dl5eu_mpeg_ts_monitor_0 = dl5eu.mpeg_ts_monitor(1000, '', False, False)
        self.dl5eu_dvbt_viterbi_decoder_0 = dl5eu.dvbt_viterbi_decoder(dtv.MOD_16QAM, dtv.NH, dtv.C2_3, 768, False)
        self.dl5eu_dvbt_tps_decoder_0 = dl5eu.dvbt_tps_decoder(dtv.T2k, True)
        self.dl5eu_dvbt_ofdm_synchronization_0 = dl5eu.dvbt_ofdm_synchronization(dtv.T2k, dtv.GI_1_32, 10, True, sync_sample_rate, squelch_threshold, True)
        self.blocks_vector_to_stream_2_0 = blocks.vector_to_stream(gr.sizeof_gr_complex*1, 1512)
        self.blocks_vector_to_stream_1 = blocks.vector_to_stream(gr.sizeof_char*1, 1512)
        self.blocks_selector_1 = blocks.selector(gr.sizeof_gr_complex*1,decim_select,0)
        self.blocks_selector_1.set_enabled(True)
        self.blocks_selector_0 = blocks.selector(gr.sizeof_gr_complex*1,0,decim_select)
        self.blocks_selector_0.set_enabled(True)
        self.blocks_null_sink_0_0 = blocks.null_sink(gr.sizeof_gr_complex*1705)
        self.blocks_file_sink_0 = blocks.file_sink(gr.sizeof_char*1, 'D:\\Projekte\\GNURadio\\video.ts', False)
        self.blocks_file_sink_0.set_unbuffered(False)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.blocks_selector_0, 0), (self.rational_resampler_xxx_0, 0))
        self.connect((self.blocks_selector_0, 1), (self.rational_resampler_xxx_1, 0))
        self.connect((self.blocks_selector_0, 2), (self.rational_resampler_xxx_2, 0))
        self.connect((self.blocks_selector_0, 3), (self.rational_resampler_xxx_2_0, 0))
        self.connect((self.blocks_selector_0, 4), (self.rational_resampler_xxx_3, 0))
        self.connect((self.blocks_selector_0, 6), (self.rational_resampler_xxx_3_0, 0))
        self.connect((self.blocks_selector_0, 5), (self.rational_resampler_xxx_4, 0))
        self.connect((self.blocks_selector_1, 0), (self.dl5eu_dvbt_ofdm_synchronization_0, 0))
        self.connect((self.blocks_vector_to_stream_1, 0), (self.dl5eu_dvbt_viterbi_decoder_0, 0))
        self.connect((self.blocks_vector_to_stream_2_0, 0), (self.qtgui_const_sink_x_0, 0))
        self.connect((self.dl5eu_dvbt_ofdm_synchronization_0, 1), (self.blocks_null_sink_0_0, 0))
        self.connect((self.dl5eu_dvbt_ofdm_synchronization_0, 0), (self.dl5eu_dvbt_tps_decoder_0, 0))
        self.connect((self.dl5eu_dvbt_ofdm_synchronization_0, 2), (self.freq_error, 0))
        self.connect((self.dl5eu_dvbt_ofdm_synchronization_0, 3), (self.samp_error, 0))
        self.connect((self.dl5eu_dvbt_tps_decoder_0, 0), (self.blocks_vector_to_stream_2_0, 0))
        self.connect((self.dl5eu_dvbt_tps_decoder_0, 0), (self.dtv_dvbt_demap_0, 0))
        self.connect((self.dl5eu_dvbt_viterbi_decoder_0, 0), (self.dtv_dvbt_convolutional_deinterleaver_0, 0))
        self.connect((self.dl5eu_mpeg_ts_monitor_0, 0), (self.blocks_file_sink_0, 0))
        self.connect((self.dl5eu_mpeg_ts_monitor_0, 0), (self.network_udp_sink_0, 0))
        self.connect((self.dtv_dvbt_bit_inner_deinterleaver_0, 0), (self.blocks_vector_to_stream_1, 0))
        self.connect((self.dtv_dvbt_convolutional_deinterleaver_0, 0), (self.dtv_dvbt_reed_solomon_dec_0, 0))
        self.connect((self.dtv_dvbt_demap_0, 0), (self.dtv_dvbt_symbol_inner_interleaver_0, 0))
        self.connect((self.dtv_dvbt_energy_descramble_0, 0), (self.dl5eu_mpeg_ts_monitor_0, 0))
        self.connect((self.dtv_dvbt_reed_solomon_dec_0, 0), (self.dtv_dvbt_energy_descramble_0, 0))
        self.connect((self.dtv_dvbt_symbol_inner_interleaver_0, 0), (self.dtv_dvbt_bit_inner_deinterleaver_0, 0))
        self.connect((self.iio_pluto_source_0, 0), (self.blocks_selector_0, 0))
        self.connect((self.rational_resampler_xxx_0, 0), (self.blocks_selector_1, 0))
        self.connect((self.rational_resampler_xxx_1, 0), (self.blocks_selector_1, 1))
        self.connect((self.rational_resampler_xxx_2, 0), (self.blocks_selector_1, 2))
        self.connect((self.rational_resampler_xxx_2_0, 0), (self.blocks_selector_1, 3))
        self.connect((self.rational_resampler_xxx_3, 0), (self.blocks_selector_1, 4))
        self.connect((self.rational_resampler_xxx_3_0, 0), (self.blocks_selector_1, 6))
        self.connect((self.rational_resampler_xxx_4, 0), (self.blocks_selector_1, 5))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("gnuradio/flowgraphs", "dvbt_rx_rtlsdr_ext")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_conf_decim_select(self):
        return self.conf_decim_select

    def set_conf_decim_select(self, conf_decim_select):
        self.conf_decim_select = conf_decim_select
        self.set_decim_select(self.conf_decim_select)

    def get_decim_select(self):
        return self.decim_select

    def set_decim_select(self, decim_select):
        self.decim_select = decim_select
        self._conf_decim_select_config = configparser.ConfigParser()
        self._conf_decim_select_config.read('config.ini')
        if not self._conf_decim_select_config.has_section('setup'):
        	self._conf_decim_select_config.add_section('setup')
        self._conf_decim_select_config.set('setup', 'decim_select', str(self.decim_select))
        self._conf_decim_select_config.write(open('config.ini', 'w'))
        self._decim_select_callback(self.decim_select)
        self.set_sync_sample_rate(2400000 * [20/21, 10/21, 5/21, 160/1009, 5/42, 5/84, 280/8909][self.decim_select])
        self.blocks_selector_0.set_output_index(self.decim_select)
        self.blocks_selector_1.set_input_index(self.decim_select)

    def get_conf_rf_gain_db(self):
        return self.conf_rf_gain_db

    def set_conf_rf_gain_db(self, conf_rf_gain_db):
        self.conf_rf_gain_db = conf_rf_gain_db
        self.set_rf_gain_db(self.conf_rf_gain_db)

    def get_conf_if_gain_db(self):
        return self.conf_if_gain_db

    def set_conf_if_gain_db(self, conf_if_gain_db):
        self.conf_if_gain_db = conf_if_gain_db
        self.set_if_gain_db(self.conf_if_gain_db)

    def get_conf_freq_corr_ppm(self):
        return self.conf_freq_corr_ppm

    def set_conf_freq_corr_ppm(self, conf_freq_corr_ppm):
        self.conf_freq_corr_ppm = conf_freq_corr_ppm
        self.set_freq_corr_ppm(self.conf_freq_corr_ppm)

    def get_conf_center_freq(self):
        return self.conf_center_freq

    def set_conf_center_freq(self, conf_center_freq):
        self.conf_center_freq = conf_center_freq
        self.set_center_freq(self.conf_center_freq)

    def get_sync_sample_rate(self):
        return self.sync_sample_rate

    def set_sync_sample_rate(self, sync_sample_rate):
        self.sync_sample_rate = sync_sample_rate
        self.dl5eu_dvbt_ofdm_synchronization_0.set_sample_rate(self.sync_sample_rate)

    def get_squelch_threshold(self):
        return self.squelch_threshold

    def set_squelch_threshold(self, squelch_threshold):
        self.squelch_threshold = squelch_threshold
        self.dl5eu_dvbt_ofdm_synchronization_0.set_squelch_threshold(self.squelch_threshold)

    def get_rf_gain_db(self):
        return self.rf_gain_db

    def set_rf_gain_db(self, rf_gain_db):
        self.rf_gain_db = rf_gain_db
        self._conf_rf_gain_db_config = configparser.ConfigParser()
        self._conf_rf_gain_db_config.read('config.ini')
        if not self._conf_rf_gain_db_config.has_section('setup'):
        	self._conf_rf_gain_db_config.add_section('setup')
        self._conf_rf_gain_db_config.set('setup', 'rf_gain_db', str(self.rf_gain_db))
        self._conf_rf_gain_db_config.write(open('config.ini', 'w'))
        self.iio_pluto_source_0.set_gain(0, self.rf_gain_db)

    def get_if_gain_db(self):
        return self.if_gain_db

    def set_if_gain_db(self, if_gain_db):
        self.if_gain_db = if_gain_db
        self._conf_if_gain_db_config = configparser.ConfigParser()
        self._conf_if_gain_db_config.read('config.ini')
        if not self._conf_if_gain_db_config.has_section('setup'):
        	self._conf_if_gain_db_config.add_section('setup')
        self._conf_if_gain_db_config.set('setup', 'if_gain_db', str(self.if_gain_db))
        self._conf_if_gain_db_config.write(open('config.ini', 'w'))

    def get_freq_corr_ppm(self):
        return self.freq_corr_ppm

    def set_freq_corr_ppm(self, freq_corr_ppm):
        self.freq_corr_ppm = freq_corr_ppm
        self._conf_freq_corr_ppm_config = configparser.ConfigParser()
        self._conf_freq_corr_ppm_config.read('config.ini')
        if not self._conf_freq_corr_ppm_config.has_section('setup'):
        	self._conf_freq_corr_ppm_config.add_section('setup')
        self._conf_freq_corr_ppm_config.set('setup', 'freq_corr_ppm', str(self.freq_corr_ppm))
        self._conf_freq_corr_ppm_config.write(open('config.ini', 'w'))

    def get_center_freq(self):
        return self.center_freq

    def set_center_freq(self, center_freq):
        self.center_freq = center_freq
        self._conf_center_freq_config = configparser.ConfigParser()
        self._conf_center_freq_config.read('config.ini')
        if not self._conf_center_freq_config.has_section('setup'):
        	self._conf_center_freq_config.add_section('setup')
        self._conf_center_freq_config.set('setup', 'center_freq', str(self.center_freq))
        self._conf_center_freq_config.write(open('config.ini', 'w'))
        self.iio_pluto_source_0.set_frequency(self.center_freq)




def main(top_block_cls=dvbt_rx_rtlsdr_ext, options=None):

    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

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
