/* -*- c++ -*- */
/*
 * Copyright 2025 Ralf Gorholt.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * =========================================================
 *  This block is based on the block "tmcc_decoder" that
 *  Federico La Rocca and others have developed for ISDB-T.
 * =========================================================
 */

#ifndef INCLUDED_DL5EU_DVBT_TPS_DECODER_IMPL_H
#define INCLUDED_DL5EU_DVBT_TPS_DECODER_IMPL_H

#include "dvbt_configure.h"
#include "dvbt_pilot_gen.h"
#include <gnuradio/dl5eu/dvbt_tps_decoder.h>

namespace gr {
namespace dl5eu {

class dvbt_tps_decoder_impl : public dvbt_tps_decoder
{
    typedef struct {
        uint8_t frame_number;
        uint8_t constellation;
        uint8_t hierarchy_info;
        uint8_t code_rate_hp;
        uint8_t code_rate_lp;
        uint8_t guard_interval;
        uint8_t trans_mode;
        uint16_t cell_id;
    } tps_info_t;

    // The configuration object of this class.
    // Should be first to be initialized first.
    const dvbt_configure d_config;

private:
    // The pilot generator object. Needed to get access
    // to pilot data, e.g. the pilot tables.
    dvbt_pilot_gen d_pg;

    // Data types used when decoding the TPS carriers
    typedef enum {
        RATE_1_2,
        RATE_2_3,
        RATE_3_4,
        RATE_5_6,
        RATE_7_8,
        C_UNUSED = 7
    } coderate_t;

    typedef enum {
        GUARD_1_32,
        GUARD_1_16,
        GUARD_1_8,
        GUARD_1_4,
        G_UNUSED = 7
    } guard_interval_t;

    typedef enum { NH, ALPHA_1, ALPHA_2, ALPHA_4 } hierarchy_t;

    typedef enum { QPSK, QAM16, QAM64, M_UNUSED = 7 } modulation_scheme_t;

    int d_init;

    // Active carriers
    const int d_num_carriers;

    // Data carriers
    const int d_num_data_carriers;

    // Continual pilots
    const int d_num_cpilots;
    const int* d_cpilots;

    // Scattered pilots
    const int d_num_tps_carriers;
    const int* d_tps_carriers;

    // TPS sync data
    const int d_tps_sync_size;
    const int* d_tps_sync_even;
    const int* d_tps_sync_odd;

    const int d_symbols_per_frame;
    const int d_frames_per_superframe;

    // Keeps the received TPS data in a FIFO
    // TODO circular_buffers are superior to deques.
    // We should migrate them all.
    std::deque<char> d_rcv_tps_data;
    // Keeps the TPS sync sequence
    std::deque<char> d_tps_sync_evenv;
    std::deque<char> d_tps_sync_oddv;

    // TPS parity check matrix
    static const char d_h[];

    // Whether or not the decoded TPS parameters should be printed on the console.
    bool d_print_tps_data;

    // Keep TPS carrier values from previous symbol
    volk::vector<gr_complex> d_prev_tps_symbol;

    // Indicates whether the last symbol was a frame end
    bool d_frame_end;
    // Indicates whther a re-sync should be signaled downstream
    bool d_resync;

    // Symbol Index
    int d_rel_symbol_index;
    int d_prev_rel_symbol_index;
    int d_symbol_index;
    int d_prev_symbol_index;

    int d_frame_index;
    int d_prev_frame_index;

    // how many symbols ago we saw a complete tps
    int d_since_last_tps;

    // A list of the data carriers for the current configuration (mode), one
    // for each symbol in a frame.
    volk::vector<int> d_data_carriers;

    bool d_sync_start;
    bool d_frame_sync;

    tps_info_t d_tps_info;
    tps_info_t d_prev_tps_info;

    // Ports for message passing
    pmt::pmt_t d_mp_mod_scheme;
    pmt::pmt_t d_mp_trans_mode;
    pmt::pmt_t d_mp_code_rate;
    pmt::pmt_t d_mp_guard_int;

    /*!
     * \brief This method creates the data carriers list that contains the list of
     * carriers that carry actual DTV data (payload carriers).
     */
    void create_data_carrier_list();

    /*!
     * \brief The method decodes the TPS carriers of a given symbol, thus appending a new
     * bit to those received so far. If 68 were received since the last complete tps, we
     * check whether the sync word is present at the beginning, and if it passes the BCH
     * parity code.
     *
     * Our present implementation simply performs the differential demodulation (TPS is
     * transmitted in DBPSK) for every carrier, and then performs a majority vote between
     * all carriers to verify which bit was sent.
     */
    bool process_tps_data(const gr_complex* in);

    void print_coderate(coderate_t coderate);

    void print_guard_interval(guard_interval_t guard_interval);

    void print_hierarchy(hierarchy_t hierarchy);

    void print_modulation_scheme(modulation_scheme_t modulation_scheme);

    void print_tps_info(const tps_info_t& tps_info);

public:
    dvbt_tps_decoder_impl(dvbt_transmission_mode_t transmission_mode,
                          bool print_tps_data);
    ~dvbt_tps_decoder_impl();

    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);
};

} // namespace dl5eu
} // namespace gr

#endif /* INCLUDED_DL5EU_DVBT_TPS_DECODER_IMPL_H */
