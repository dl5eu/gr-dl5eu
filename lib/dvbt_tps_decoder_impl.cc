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

#include "dvbt_tps_decoder_impl.h"
#include <gnuradio/io_signature.h>
#include <cstdio>
#include <cinttypes>

namespace gr {
namespace dl5eu {

static const char* blockid = "dvbt_tps_decoder";

namespace {
/*!
 * @brief Return the number of active carriers, including pilots and TPS.
 * @param transmission_mode Configured 2K or 8K FFT mode (fallback: 2K).
 * @return Complex samples per corresponding stream vector item.
 */
int get_num_carriers(dvbt_transmission_mode_t transmission_mode)
{
    int num_carriers = 0;

    switch (transmission_mode) {
    case T8k:
        num_carriers = 6817;
        break;
    case T2k:
    default:
        num_carriers = 1705;
        break;
    }

    return num_carriers;
}
/*!
 * @brief Return the number of payload carriers after removing pilots and TPS.
 * @param transmission_mode Configured 2K or 8K FFT mode (fallback: 2K).
 * @return Complex samples per corresponding stream vector item.
 */
int get_num_payload_carriers(dvbt_transmission_mode_t transmission_mode)
{
    int num_carriers = 0;

    switch (transmission_mode) {
    case T8k:
        num_carriers = 6048;
        break;
    case T2k:
    default:
        num_carriers = 1512;
        break;
    }

    return num_carriers;
}
} // namespace

using input_type = gr_complex;
using output_type = gr_complex;

void dvbt_tps_decoder_impl::create_data_carrier_list()
{
    for (int symbol_index = 0; symbol_index < 4; ++symbol_index) {
        int cpilot_index = 0;
        int spilot_index = 0;
        int tps_pilot_index = 0;
        int carrier_out = 0;

        for (int carrier = 0; carrier < d_num_carriers - 1; ++carrier) {
            if (carrier == (12 * spilot_index + 3 * (symbol_index % 4))) {
                // The current carrier is a scattered pilot
                ++spilot_index;
                if (carrier == d_cpilots[cpilot_index]) {
                    // The current scattered pilot is at a continual pilot's position
                    ++cpilot_index;
                }
            } else if (carrier == d_cpilots[cpilot_index]) {
                // The current carrier is a continual pilot
                ++cpilot_index;
            } else if (carrier == d_tps_carriers[tps_pilot_index]) {
                // The current carrier is a TPS pilot
                ++tps_pilot_index;
            } else {
                d_data_carriers[symbol_index * d_num_data_carriers + carrier_out] =
                    carrier;
                ++carrier_out;
            }
        }
    }
}

bool dvbt_tps_decoder_impl::process_tps_data(const gr_complex* in,
                                             const int diff_symbol_index)
{
    bool end_of_frame = false;

    /*
     * We first check whether the TPS carriers of this OFDM symbol carry a 0 or a 1.
     * We have tps_carriers_size TPS carriers. TPS carriers use majority voting for
     * decision.
     */
    int tps_majority_zero = 0;

    // For every TPS carrier in the symbol...
    for (int k = 0; k < d_num_tps_carriers; ++k) {
        // As the modulation is DBPSK, we compare the current TPS carrier value to the
        // previous one
        gr_complex phdiff = in[d_tps_carriers[k]] * conj(d_prev_tps_symbol[k]);

        if (phdiff.real() >= 0.0) {
            ++tps_majority_zero;
        } else {
            --tps_majority_zero;
        }
        // Keep the current carrier for the next comparison
        d_prev_tps_symbol[k] = in[d_tps_carriers[k]];
    }

    // Preserve TPS bit spacing when pilot phase indicates missing positions.
    // Zero placeholders do not recover missing data: the BCH helper only checks
    // parity here. A symbol discontinuity is handled by the resync path.
    // Symbol zero is the DBPSK reference and carries no differential TPS data.
    for (int i = 0; i < diff_symbol_index; i++) {
        // Delete first element (we keep the last 67 bits)
        d_rcv_tps_data.pop_front();

        // Add data at tail
        if (i < diff_symbol_index - 1 ||
            (d_symbol_index_known && d_symbol_index == 0)) {
            d_rcv_tps_data.push_back(0);
        } else {
            d_rcv_tps_data.push_back(tps_majority_zero >= 0 ? 0 : 1);
        }
    }

    /*
     * While synchronized, the TPS frame boundary is determined by the known
     * 68-symbol timing, not by a successful BCH check. One damaged TPS frame
     * must not interrupt the payload stream or shift its frame_start tags.
     * Before synchronization, continue searching for a valid TPS frame in the
     * sliding bit FIFO.
     */
    const bool locked = d_symbol_index_known && !d_sync_start && !d_resync;
    const bool expected_frame_end =
        locked && d_symbol_index == d_symbols_per_frame - 1;

    if (locked && !expected_frame_end) {
        return false;
    }

    const bool sync_even = std::equal(d_rcv_tps_data.begin() + 1,
                                     d_rcv_tps_data.begin() + d_tps_sync_size,
                                     d_tps_sync_evenv.begin());
    const bool sync_odd = std::equal(d_rcv_tps_data.begin() + 1,
                                    d_rcv_tps_data.begin() + d_tps_sync_size,
                                    d_tps_sync_oddv.begin());
    const bool sync_valid = sync_even || sync_odd;

    if (!expected_frame_end && !sync_valid) {
        return false;
    }

    const bool bch_valid = sync_valid && d_pg.verify_bch_code(d_rcv_tps_data) == 0;
    const int received_frame_index = (d_rcv_tps_data[23] << 1) | d_rcv_tps_data[24];

    if (locked && bch_valid &&
        (received_frame_index != d_frame_index ||
         sync_even != ((d_frame_index % 2) == 0))) {
        // A BCH-validated frame contradicts the tracked frame sequence.
        // This is evidence of lost frame alignment, not an isolated bit error.
        std::fprintf(stderr,
                     "TPS_RESYNC: reason=FRAME_INDEX_MISMATCH, input_symbol=%" PRIu64
                     ", expected_frame=%d, received_frame=%d\n",
                     d_diag_input_symbols, d_frame_index, received_frame_index);
        d_symbol_index_known = false;
        d_tps_complete = false;
        d_diag_frame_phase_valid = false;
        d_consecutive_tps_errors = 0;
        d_resync = true;
    } else if (bch_valid) {
        d_consecutive_tps_errors = 0;
        d_frame_index = received_frame_index;
        d_symbol_index_known = true;
        end_of_frame = true;
        decode_tps_data();
    } else {
        if (sync_valid) {
            ++d_diag_bch_failures;
        }

        if (locked) {
            ++d_consecutive_tps_errors;
            if (d_consecutive_tps_errors == 1) {
                // The OFDM timing is still known. Advance the frame index on
                // the next symbol and keep the last validated TPS parameters.
                end_of_frame = true;
                std::fprintf(stderr,
                             "TPS_FRAME_INVALID: reason=%s, input_symbol=%" PRIu64
                             ", frame=%d, continuing=1\n",
                             sync_valid ? "BCH_FAILED" : "SYNC_WORD_INVALID",
                             d_diag_input_symbols, d_frame_index);
            } else {
                std::fprintf(stderr,
                             "TPS_RESYNC: reason=CONSECUTIVE_TPS_ERRORS, input_symbol=%" PRIu64
                             ", count=%u\n",
                             d_diag_input_symbols, d_consecutive_tps_errors);
                d_symbol_index_known = false;
                d_tps_complete = false;
                d_diag_frame_phase_valid = false;
                d_resync = true;
            }
        } else {
            // No reliable frame timing yet: retain the acquisition behaviour.
            d_symbol_index_known = false;
            d_tps_complete = false;
            d_diag_frame_phase_valid = false;
            d_resync = true;
            d_consecutive_tps_errors = 0;
        }
    }

    // Begin collecting the next TPS frame (or a fresh acquisition window).
    for (int i = 0; i < d_symbols_per_frame; ++i) {
        d_rcv_tps_data[i] = 0;
    }

    #if 0
    if (end_of_frame) {
        std::fprintf(stderr,
                    "TPS_FRAME_OK: frame=%d, complete=%d, t_ms=%lld\n",
                    d_frame_index,
                    static_cast<int>(d_tps_complete),
                    diagnostic_time_ms());
    }
    #endif

    return end_of_frame;
}

void dvbt_tps_decoder_impl::decode_tps_data()
{
    // Stage the two cell-ID halves until the odd frame completes the pair.
    // This function-local staging object is shared across decoder instances.
    static tps_info_t tps_info{};

    // Frame number
    tps_info.frame_index = (d_rcv_tps_data[23] << 1) | d_rcv_tps_data[24];
    // Constellation
    tps_info.constellation = (d_rcv_tps_data[25] << 1) | d_rcv_tps_data[26];
    // Hierarchy information
    tps_info.hierarchy_info =
        (d_rcv_tps_data[27] << 2) | (d_rcv_tps_data[28] << 1) | d_rcv_tps_data[29];
    // Code rate HP
    tps_info.code_rate_hp =
        (d_rcv_tps_data[30] << 2) | (d_rcv_tps_data[31] << 1) | d_rcv_tps_data[32];
    // Code rate LP
    tps_info.code_rate_lp =
        (d_rcv_tps_data[33] << 2) | (d_rcv_tps_data[34] << 1) | d_rcv_tps_data[35];
    // Guard interval
    tps_info.guard_interval = (d_rcv_tps_data[36] << 1) | d_rcv_tps_data[37];
    // Transmission mode
    tps_info.trans_mode = (d_rcv_tps_data[38] << 1) | d_rcv_tps_data[39];
    // Cell ID
    uint8_t cell_id = 0;
    if (tps_info.frame_index % 2) {
        // Odd frame (1 or 3)
        cell_id = (d_rcv_tps_data[40] << 7) | (d_rcv_tps_data[41] << 6) |
                  (d_rcv_tps_data[42] << 5) | (d_rcv_tps_data[43] << 4) |
                  (d_rcv_tps_data[44] << 3) | (d_rcv_tps_data[45] << 2) |
                  (d_rcv_tps_data[46] << 1) | (d_rcv_tps_data[47]);
        tps_info.cell_id &= 0x00ff;
        tps_info.cell_id |= cell_id << 8;
        d_tps_info = tps_info;
        d_tps_complete = true;
    } else {
        // Even frame (0 or 2)
        cell_id = (d_rcv_tps_data[40] << 7) | (d_rcv_tps_data[41] << 6) |
                  (d_rcv_tps_data[42] << 5) | (d_rcv_tps_data[43] << 4) |
                  (d_rcv_tps_data[44] << 3) | (d_rcv_tps_data[45] << 2) |
                  (d_rcv_tps_data[46] << 1) | (d_rcv_tps_data[47]);
        tps_info.cell_id = cell_id;
        d_tps_complete = false;
    }
}

void dvbt_tps_decoder_impl::print_coderate(coderate_t coderate)
{
    printf("Convolutional Code Rate: ");

    switch (coderate) {
    case RATE_1_2:
        printf("1/2\n");
        break;
    case RATE_2_3:
        printf("2/3\n");
        break;
    case RATE_3_4:
        printf("3/4\n");
        break;
    case RATE_5_6:
        printf("5/6\n");
        break;
    case RATE_7_8:
        printf("7/8\n");
        break;
    case C_UNUSED:
        printf("UNUSED\n");
        break;
    default:
        printf("TPS Decoder: Error: something went wrong while decoding the "
               "Convolutional Code Rate.\n");
    }
}

void dvbt_tps_decoder_impl::print_guard_interval(guard_interval_t guard_interval)
{
    printf("Guard Interval: ");

    switch (guard_interval) {
    case GUARD_1_4:
        printf("1/4\n");
        break;
    case GUARD_1_8:
        printf("1/8\n");
        break;
    case GUARD_1_16:
        printf("1/16\n");
        break;
    case GUARD_1_32:
        printf("1/32\n");
        break;
    case G_UNUSED:
        printf("UNUSED\n");
        break;
    default:
        printf("TPS Decoder: Error: something went wrong while decoding the "
               "Guard Interval.\n");
    }
}

void dvbt_tps_decoder_impl::print_hierarchy(hierarchy_t hierarchy)
{
    printf("Hierachy: ");

    switch (hierarchy) {
    case NH:
        printf("No Hierarchy\n");
        break;
    case ALPHA_1:
        printf("Alpha 1\n");
        break;
    case ALPHA_2:
        printf("Alpha 2\n");
        break;
    case ALPHA_4:
        printf("Alpha 4\n");
        break;
    default:
        printf("TPS Decoder: Error: something went wrong while decoding the "
               "Hierarchy.\n");
    }
}

void dvbt_tps_decoder_impl::print_modulation_scheme(modulation_scheme_t modulation_scheme)
{
    printf("Carrier Modulation Scheme: ");

    switch (modulation_scheme) {
    case QPSK:
        printf("QPSK\n");
        break;
    case QAM16:
        printf("16QAM\n");
        break;
    case QAM64:
        printf("64QAM\n");
        break;
    case M_UNUSED:
        printf("UNUSED\n");
        break;
    default:
        printf("TPS Decoder: Error: something went wrong while decoding the "
               "Carrier Modulation Scheme.\n");
    }
}

void dvbt_tps_decoder_impl::print_tps_info(const tps_info_t& tps_info)
{
    printf("\n--- TPS Information ---\n");

    print_modulation_scheme((modulation_scheme_t)tps_info.constellation);

    print_hierarchy((hierarchy_t)tps_info.hierarchy_info);
    printf("HP ");
    print_coderate((coderate_t)tps_info.code_rate_hp);

    printf("LP ");
    print_coderate((coderate_t)tps_info.code_rate_lp);

    print_guard_interval((guard_interval_t)tps_info.guard_interval);

    printf("Cell ID: %d\n\n", d_tps_info.cell_id);
}

dvbt_tps_decoder::sptr dvbt_tps_decoder::make(dvbt_transmission_mode_t transmission_mode,
                                              bool print_tps_data)
{
    return gnuradio::make_block_sptr<dvbt_tps_decoder_impl>(transmission_mode,
                                                            print_tps_data);
}

dvbt_tps_decoder_impl::dvbt_tps_decoder_impl(dvbt_transmission_mode_t transmission_mode,
                                             bool print_tps_data)
    : gr::block(
          "dvbt_tps_decoder",
          gr::io_signature::make(
              1, 1, sizeof(input_type) * get_num_carriers(transmission_mode)),
          gr::io_signature::make(
              1, 1, sizeof(output_type) * get_num_payload_carriers(transmission_mode))),
      d_config{ dtv::MOD_16QAM, dtv::NH,      dtv::C1_2,
                dtv::C1_2,      dtv::GI_1_32, transmission_mode },
      d_pg{ d_config },
      d_init{ 0 },
      d_num_carriers{ d_config.d_Kmax - d_config.d_Kmin + 1 },
      d_num_data_carriers{ d_config.d_payload_length },
      d_num_cpilots{ d_pg.cpilot_carriers_size() },
      d_cpilots{ d_pg.cpilot_carriers() },
      d_num_tps_carriers{ d_pg.tps_carriers_size() },
      d_tps_carriers{ d_pg.tps_carriers() },
      d_tps_sync_size{ d_pg.tps_sync_size() },
      d_tps_sync_even{ d_pg.tps_sync_even() },
      d_tps_sync_odd{ d_pg.tps_sync_odd() },
      d_symbols_per_frame{ d_config.d_symbols_per_frame },
      d_frames_per_superframe{ d_config.d_frames_per_superframe },
      d_prev_tps_symbol(d_num_tps_carriers),
      d_frame_end{ false },
      d_resync{ false },
      d_rel_symbol_index{ 0 },
      d_prev_rel_symbol_index{ 0 },
      d_symbol_index{ 0 },
      d_prev_symbol_index{ 0 },
      d_symbol_index_known{ false },
      d_frame_index{ 0 },
      d_prev_frame_index{ 0 },
      d_tps_complete{ false },
      d_data_carriers(4 * d_num_data_carriers),
      d_sync_start{ true },
      d_tps_info{},
      // Message ports
      d_mp_mod_scheme{ pmt::intern("const") },
      d_mp_trans_mode{ pmt::intern("mode") },
      d_mp_code_rate{ pmt::intern("fec") },
      d_mp_guard_int{ pmt::intern("guard") }
{
    create_data_carrier_list();

    d_print_tps_data = print_tps_data;

    memset(static_cast<void*>(&d_prev_tps_symbol[0]),
           0,
           d_num_tps_carriers * sizeof(gr_complex));

    // Init TPS sync sequence
    for (int i = 0; i < d_tps_sync_size; ++i) {
        d_tps_sync_evenv.push_back(d_tps_sync_even[i]);
        d_tps_sync_oddv.push_back(d_tps_sync_odd[i]);
    }

    // Init receive TPS data vector
    for (int i = 0; i < d_symbols_per_frame; ++i) {
        d_rcv_tps_data.push_back(0);
    }

    // Register message ports
    message_port_register_out(d_mp_mod_scheme);
    message_port_register_out(d_mp_trans_mode);
    message_port_register_out(d_mp_code_rate);
    message_port_register_out(d_mp_guard_int);

    // Acquisition drops input symbols; automatic tag propagation would leave
    // downstream frame offsets inconsistent with the shorter output stream.
    set_tag_propagation_policy(TPP_DONT);
}

dvbt_tps_decoder_impl::~dvbt_tps_decoder_impl() {}

void dvbt_tps_decoder_impl::forecast(int noutput_items,
                                     gr_vector_int& ninput_items_required)
{
    int ninputs = ninput_items_required.size();

    for (int i = 0; i < ninputs; ++i) {
        ninput_items_required[i] = noutput_items;
    }
}

int dvbt_tps_decoder_impl::general_work(int noutput_items,
                                        gr_vector_int& ninput_items,
                                        gr_vector_const_void_star& input_items,
                                        gr_vector_void_star& output_items)
{
    auto in = static_cast<const input_type*>(input_items[0]);
    auto out = static_cast<output_type*>(output_items[0]);

    std::vector<tag_t> tags;

    for (int i = 0; i < noutput_items; ++i) {
        // Upstream sync_start invalidates prior frame alignment. Payload must
        // wait for TPS acquisition even if the OFDM samples now look usable.
        this->get_tags_in_window(tags, 0, i, i + 1, pmt::string_to_symbol("sync_start"));
        if (!tags.empty()) {
            d_diag_frame_phase_valid = false;
            if (!d_sync_start) {
                d_sync_start = true;
                printf("TPS Decoder: general_work(): %s received from %s\n",
                       pmt::symbol_to_string(tags[0].key).c_str(),
                       pmt::symbol_to_string(tags[0].srcid).c_str());
            }
        }

        // We check if the block upstream signaled us to re-sync, e.g. because one
        // or more symbols have been lost.
        this->get_tags_in_window(tags, 0, i, i + 1, pmt::string_to_symbol("resync"));
        if (!tags.empty()) {
            d_diag_frame_phase_valid = false;
            if (!d_resync) {
                d_resync = true;
                printf("TPS Decoder: general_work(): %s received from %s\n",
                       pmt::symbol_to_string(tags[0].key).c_str(),
                       pmt::symbol_to_string(tags[0].srcid).c_str());
            }
        }

        const bool frame_start = d_frame_end;
        const bool frame_resync = frame_start && (d_sync_start || d_resync);
        if (frame_start) {
            // Being here means that the previous OFDM symbol constituted the end of a
            // frame. The current symbol is the first symbol of the next frame!
            d_frame_index = (d_frame_index + 1) % d_frames_per_superframe;
            d_symbol_index = 0;
            d_rel_symbol_index = 0;
        } else {
            d_symbol_index = (d_symbol_index + 1) % d_symbols_per_frame;
            d_rel_symbol_index = (d_rel_symbol_index + 1) % 4;
        }

        // The OFDM synchronizer supplies the scattered-pilot phase for every
        // symbol. Use it for data-carrier extraction and TPS symbol spacing.
        // The TPS frame counter remains independent so inconsistent phases can
        // still be detected instead of silently masking a lost symbol.
        d_diag_input_symbols = nitems_read(0) + i;
        this->get_tags_in_window(
            tags, 0, i, i + 1, pmt::string_to_symbol("relative_symbol_index"));
        if (tags.empty()) {
            ++d_diag_missing_phase_tags;
            if (d_diag_missing_phase_tags == 1 || d_diag_missing_phase_tags % 1024 == 0) {
                std::fprintf(stderr,
                             "TPS_PHASE_TAG_MISSING: input_symbol=%" PRIu64
                             ", missing_total=%" PRIu64 "\n",
                             d_diag_input_symbols, d_diag_missing_phase_tags);
            }
        } else if (pmt::is_integer(tags[0].value)) {
            const long received_phase = pmt::to_long(tags[0].value);
            if (received_phase < 0 || received_phase > 3) {
                std::fprintf(stderr,
                             "TPS_PHASE_TAG_INVALID: input_symbol=%" PRIu64
                             ", value=%ld\n",
                             d_diag_input_symbols, received_phase);
            } else {
                if (d_diag_frame_phase_valid && received_phase != d_rel_symbol_index) {
                    ++d_diag_phase_mismatches;
                    std::fprintf(stderr,
                                 "TPS_PHASE_MISMATCH: input_symbol=%" PRIu64
                                 ", tps=%d, ofdm=%ld, total=%" PRIu64
                                 ", forcing_resync=1\n",
                                 d_diag_input_symbols, d_rel_symbol_index,
                                 received_phase, d_diag_phase_mismatches);
                    d_resync = true;
                    d_diag_frame_phase_valid = false;
                }
                d_rel_symbol_index = static_cast<int>(received_phase);
            }
        } else {
            std::fprintf(stderr,
                         "TPS_PHASE_TAG_INVALID: input_symbol=%" PRIu64
                         ", non-integer value\n", d_diag_input_symbols);
        }

        int diff_rel_symbol_index =
            (d_rel_symbol_index - d_prev_rel_symbol_index + 4) % 4;
        d_prev_rel_symbol_index = d_rel_symbol_index;
        if (diff_rel_symbol_index != 1) {
            if (!(d_sync_start | d_resync)) {
                d_resync = true;
                printf("TPS Decoder: general_work(): One or more symbols lost.\n");
            }
        }

        // Process and decode the TPS data
        d_frame_end = process_tps_data(&in[i * d_num_carriers], diff_rel_symbol_index);

        if (d_sync_start || d_resync) {
            if (d_symbol_index_known && d_symbol_index == 0 && d_frame_index == 0 &&
                d_tps_complete) {
                // This is a superframe start, we signal it downstream.
                d_sync_start = false;
                d_resync = false;
                d_diag_frame_phase_valid = true;
                std::fprintf(stderr,
                             "TPS_SYNC_ACQUIRED: input_symbol=%" PRIu64
                             ", frame=%d, bch_failures=%" PRIu64
                             ", phase_mismatches=%" PRIu64 "\n",
                             d_diag_input_symbols, d_frame_index,
                             d_diag_bch_failures, d_diag_phase_mismatches);
                // Send TPS information
                message_port_pub(
                    d_mp_mod_scheme,
                    pmt::cons(pmt::PMT_NIL, pmt::from_long(d_tps_info.constellation)));
                message_port_pub(
                    d_mp_trans_mode,
                    pmt::cons(pmt::PMT_NIL, pmt::from_long(d_tps_info.trans_mode)));
                message_port_pub(
                    d_mp_code_rate,
                    pmt::cons(pmt::PMT_NIL, pmt::from_long(d_tps_info.code_rate_hp)));
                message_port_pub(
                    d_mp_guard_int,
                    pmt::cons(pmt::PMT_NIL, pmt::from_long(d_tps_info.guard_interval)));
                if (d_print_tps_data) {
                    print_tps_info(d_tps_info);
                }
                const uint64_t offset = this->nitems_written(0) + i;
                pmt::pmt_t key = pmt::string_to_symbol("superframe_start");
                pmt::pmt_t value = pmt::from_long(0xaa);
                pmt::pmt_t srcid = pmt::string_to_symbol(blockid);
                this->add_item_tag(0, offset, key, value, srcid);
            } else {
                // Consume the rejected symbol and all preceding processed symbols.
                consume_each(i + 1);
                return i;
            }
        }

        if (frame_start) {
            const uint64_t offset = this->nitems_written(0) + i;
            // This is a frame start, we signal it downstream
            pmt::pmt_t key = pmt::string_to_symbol("frame_start");
            pmt::pmt_t value = pmt::from_long(d_frame_index);
            pmt::pmt_t srcid = pmt::string_to_symbol(blockid);
            this->add_item_tag(0, offset, key, value, srcid);

            // Forward the saved restart condition at the accepted frame boundary;
            // acquisition above may already have cleared the live resync flags.
            if (frame_resync) {
                pmt::pmt_t key = pmt::string_to_symbol("resync");
                pmt::pmt_t value = pmt::from_long(0);
                this->add_item_tag(0, offset, key, value, srcid);
            }
        }

        // Send a tag for each OFDM symbol informing about the symbol index.
        const uint64_t offset = this->nitems_written(0) + i;
        pmt::pmt_t key = pmt::string_to_symbol("symbol_index");
        pmt::pmt_t value = pmt::from_long(d_symbol_index);
        pmt::pmt_t srcid = pmt::string_to_symbol(blockid);
        this->add_item_tag(0, offset, key, value, srcid);

        // Copy the data carriers to the output buffer
        for (int dc = 0; dc < d_num_data_carriers; ++dc) {
            out[i * d_num_data_carriers + dc] =
                in[i * d_num_carriers +
                   d_data_carriers[d_num_data_carriers * d_rel_symbol_index + dc]];
        }
    }

    consume_each(noutput_items);

    return (noutput_items);
}

} /* namespace dl5eu */
} /* namespace gr */
