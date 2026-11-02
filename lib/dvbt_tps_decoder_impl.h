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
#include <cstdint>

namespace gr {
namespace dl5eu {

/*!
 * @brief Recover TPS frame timing and extract payload carriers from equalized symbols.
 *
 * Input items contain all active carriers; output items contain only payload
 * carriers. Pilot phase selects one of four extraction tables. TPS validates the
 * 68-symbol frame and four-frame superframe structure. During acquisition, input
 * symbols are consumed without output until a usable superframe boundary is known.
 * Stream tags are regenerated because dropped acquisition symbols change offsets.
 */
/*!
 * @brief Recover TPS frame timing and extract payload carriers from equalized symbols.
 *
 * Input items contain all active carriers; output items contain only payload
 * carriers. Pilot phase selects one of four extraction tables. TPS validates the
 * 68-symbol frame and four-frame superframe structure. During acquisition, input
 * symbols are consumed without output until a usable superframe boundary is known.
 * Stream tags are regenerated because dropped acquisition symbols change offsets.
 */
class dvbt_tps_decoder_impl : public dvbt_tps_decoder
{
    typedef struct {
        uint8_t frame_index;
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

    // TPS carrier positions (not scattered pilots).
    const int d_num_tps_carriers;
    const int* d_tps_carriers;

    // TPS sync data
    const int d_tps_sync_size;
    const int* d_tps_sync_even;
    const int* d_tps_sync_odd;

    const int d_symbols_per_frame;
    const int d_frames_per_superframe;

    // Sliding 68-position TPS history; position zero is the DBPSK reference.
    std::deque<char> d_rcv_tps_data;
    // Keeps the TPS sync sequence
    std::deque<char> d_tps_sync_evenv;
    std::deque<char> d_tps_sync_oddv;

    // Legacy parity-table declaration; validation is delegated to d_pg.
    static const char d_h[];

    // Whether or not the decoded TPS parameters should be printed on the console.
    bool d_print_tps_data;

    // Keep TPS carrier values from previous symbol
    volk::vector<gr_complex> d_prev_tps_symbol;

    // Indicates whether the last symbol was a frame end
    bool d_frame_end;
    // Requests reacquisition before payload output resumes.
    bool d_resync;

    // Pilot phase is modulo four; absolute symbol position is modulo 68.
    int d_rel_symbol_index;
    int d_prev_rel_symbol_index;
    int d_symbol_index;
    int d_prev_symbol_index;
    bool d_symbol_index_known;

    int d_frame_index;
    int d_prev_frame_index;

    bool d_tps_complete;

    // Four payload extraction maps, indexed by scattered-pilot phase.
    volk::vector<int> d_data_carriers;

    bool d_sync_start;

    // Phase-consistency validity also gates mismatch-triggered resynchronization.
    // Counters below record absolute input symbols and observed failures.
    bool d_diag_frame_phase_valid{ false };
    uint64_t d_diag_input_symbols{ 0 };
    uint64_t d_diag_missing_phase_tags{ 0 };
    uint64_t d_diag_phase_mismatches{ 0 };
    uint64_t d_diag_bch_failures{ 0 };
    unsigned int d_consecutive_tps_errors{ 0 };

    tps_info_t d_tps_info;

    // Ports for message passing
    pmt::pmt_t d_mp_mod_scheme;
    pmt::pmt_t d_mp_trans_mode;
    pmt::pmt_t d_mp_code_rate;
    pmt::pmt_t d_mp_guard_int;

    /*!
     * @brief Precompute payload indices for each of the four scattered-pilot phases.
     *
     * Exclude continual pilots, scattered pilots and TPS carriers once during setup,
     * so the processing path only gathers preselected payload samples.
     */
    void create_data_carrier_list();

    /*!
     * @brief Differentially demodulate TPS and validate or advance frame timing.
     * @param in One equalized active-carrier vector.
     * @param diff_symbol_index Modulo-four phase advance since the previous symbol;
     * missing positions receive placeholders, without claiming to repair lost symbols.
     * @return True at an accepted frame end, including one tolerated invalid frame;
     * false otherwise, including when resynchronization is required.
     *
     * DBPSK decisions are combined by majority vote across TPS carriers. Acquisition
     * uses a sliding sync-word/BCH search. Once locked, the known 68-symbol boundary
     * survives one invalid frame without replacing validated parameters. Two successive
     * invalid frames or a BCH-valid contradictory frame number force reacquisition.
     * verify_bch_code() checks parity here; this path does not correct TPS bit errors.
     */
    bool process_tps_data(const gr_complex* in, const int diff_symbol_index);

    /*!
     * @brief Extract fields from a TPS frame that has passed sync and BCH validation.
     *
     * Assemble the low cell-ID byte from an even frame and the high byte from an odd
     * frame. On an odd frame, publish the assembled structure internally and mark it
     * complete. This is field extraction, not a BCH error-correction step.
     */
    void decode_tps_data();

    /*!
     * @brief Print the readable convolutional-code-rate label for a TPS field.
     * @param coderate Encoded TPS code-rate value.
     */
    void print_coderate(coderate_t coderate);

    /*!
     * @brief Print the readable guard-interval label for a TPS field.
     * @param guard_interval Encoded TPS guard-interval value.
     */
    void print_guard_interval(guard_interval_t guard_interval);

    /*!
     * @brief Print the readable hierarchy label for a TPS field.
     * @param hierarchy Encoded TPS hierarchy value.
     */
    void print_hierarchy(hierarchy_t hierarchy);

    /*!
     * @brief Print the readable constellation label for a TPS field.
     * @param modulation_scheme Encoded TPS constellation value.
     */
    void print_modulation_scheme(modulation_scheme_t modulation_scheme);

    /*!
     * @brief Print modulation, hierarchy, code rates, guard interval and assembled cell ID.
     * @param tps_info TPS fields to display; cell ID is read from d_tps_info.
     *
     * Called after acquisition when console display was requested.
     */
    void print_tps_info(const tps_info_t& tps_info);

public:
    /*!
     * @brief Initialize carrier maps, TPS history and parameter message ports.
     * @param transmission_mode Configured 2K/8K mode, which fixes vector dimensions.
     * @param print_tps_data Print validated TPS information when synchronization is acquired.
     *
     * The const, mode, fec and guard message ports publish encoded TPS values as PMT
     * pairs on acquisition; they do not resize this block's stream items dynamically.
     */
    dvbt_tps_decoder_impl(dvbt_transmission_mode_t transmission_mode,
                          bool print_tps_data);
    ~dvbt_tps_decoder_impl();

    /*!
     * @brief Request one active-carrier input vector per requested payload vector.
     * @param noutput_items Requested output symbol count.
     * @param ninput_items_required Receives the corresponding input vector count.
     *
     * Acquisition may discard symbols, so a work call can produce fewer than requested.
     */
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    /*!
     * @brief Track frame timing, publish synchronization tags and gather payload carriers.
     * @param noutput_items Available output-vector capacity.
     * @param ninput_items Scheduler-supplied input vector counts.
     * @param input_items Equalized active-carrier vectors.
     * @param output_items Payload-only vectors.
     * @return Number of vectors produced; consumed acquisition symbols may exceed this.
     *
     * Use relative_symbol_index for carrier extraction, while checking its consistency
     * against independently tracked TPS timing. Regenerate symbol_index, frame_start,
     * superframe_start and resync tags at output offsets. A rejected acquisition symbol
     * is consumed exactly once, preventing repeated processing of the same input.
     */
    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);
};

} // namespace dl5eu
} // namespace gr

#endif /* INCLUDED_DL5EU_DVBT_TPS_DECODER_IMPL_H */
