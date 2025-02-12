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

namespace gr {
namespace dl5eu {

static const char* blockid = "dvbt_tps_decoder";

namespace {
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

    // Insert obtained TPS bit into the FIFO.
    // Insert a zero into FIFO in the case diff_symbol_index is greater than one. This
    // will happen in the case of losing 1 to 3 symbols. This could be corrected by BCH
    // decoder afterwards.
    for (int i = 0; i < diff_symbol_index; i++) {
        // Delete first element (we keep the last 67 bits)
        d_rcv_tps_data.pop_front();

        // Add data at tail
        if (!d_symbol_index_known || (d_symbol_index != 0)) {
            if (tps_majority_zero >= 0) {
                // If most of the symbols voted a zero, we add a zero
                d_rcv_tps_data.push_back(0);
            } else {
                // Otherwise, we add a one
                d_rcv_tps_data.push_back(1);
            }
        } else {
            d_rcv_tps_data.push_back(0);
        }
    }

    /*
     * We now check whether a complete frame has been received. To do this, we first check
     * whether the beginning of what we have received so far is equal to the 2-Byte
     * synchronization word. Then we check the BCH parity code at the end.
     */
    if (std::equal(d_rcv_tps_data.begin() + 1,
                   d_rcv_tps_data.begin() + d_tps_sync_size,
                   d_tps_sync_evenv.begin())) {
        // Verify parity for TPS data
        if (d_pg.verify_bch_code(d_rcv_tps_data) == 0) {
            // We have received a complete frame
            d_frame_index = (d_rcv_tps_data[23] << 1) | d_rcv_tps_data[24];
            d_symbol_index_known = true;
            end_of_frame = true;
            decode_tps_data();
        } else {
            d_symbol_index_known = false;
            end_of_frame = false;
            // d_sync_start = true;
            // d_resync = true;
            d_frame_sync = false;
            printf("TPS Decoder: process_tps_data(): frame error -> resync!\n");
        }
        // Clear up FIFO
        for (int i = 0; i < d_symbols_per_frame; ++i) {
            d_rcv_tps_data[i] = 0;
        }
    } else if (std::equal(d_rcv_tps_data.begin() + 1,
                          d_rcv_tps_data.begin() + d_tps_sync_size,
                          d_tps_sync_oddv.begin())) {
        // Verify parity for TPS data
        if (d_pg.verify_bch_code(d_rcv_tps_data) == 0) {
            // We have received a complete frame
            d_frame_index = (d_rcv_tps_data[23] << 1) | d_rcv_tps_data[24];
            d_symbol_index_known = true;
            end_of_frame = true;
            decode_tps_data();
        } else {
            d_symbol_index_known = false;
            end_of_frame = false;
            // d_sync_start = true;
            // d_resync = true;
            d_frame_sync = false;
            printf("TPS Decoder: process_tps_data(): frame error -> resync!\n");
        }
        // Clear up FIFO
        for (int i = 0; i < d_symbols_per_frame; ++i) {
            d_rcv_tps_data[i] = 0;
        }
    }

    // We return end_of_frame
    return end_of_frame;
}

void dvbt_tps_decoder_impl::decode_tps_data()
{
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

/*
 * The private constructor
 */
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
      d_sync_start{ false },
      d_frame_sync{ false },
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

    // No tag propagation in this block. Tags that are needed
    // by the blocks downstream are generated in this block.
    // set_tag_propagation_policy(TPP_DONT);
}

/*
 * Our virtual destructor.
 */
dvbt_tps_decoder_impl::~dvbt_tps_decoder_impl() {}

void dvbt_tps_decoder_impl::forecast(int noutput_items,
                                     gr_vector_int& ninput_items_required)
{
    int ninputs = ninput_items_required.size();

    for (int i = 0; i < ninputs; ++i) {
        ninput_items_required[i] = 2 * noutput_items;
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
        // We check if the block upstream has signaled us sync_start, which means that
        // synchronization has not been achieved yet or that it has been lost.
        this->get_tags_in_window(tags, 0, i, i + 1, pmt::string_to_symbol("sync_start"));
        if (!tags.empty()) {
            if (!d_sync_start) {
                d_sync_start = true;
                d_frame_sync = false;
                printf("TPS decoder: %s received from %s\n",
                       pmt::symbol_to_string(tags[0].key).c_str(),
                       pmt::symbol_to_string(tags[0].srcid).c_str());
            }
        }

        // We check if the block upstream signaled us to re-sync, e.g. because one
        // or more symbols have been lost.
        this->get_tags_in_window(tags, 0, i, i + 1, pmt::string_to_symbol("resync"));
        if (!tags.empty()) {
            if (!d_resync) {
                d_resync = true;
                d_frame_sync = false;
                printf("TPS decoder: %s received from %s\n",
                       pmt::symbol_to_string(tags[0].key).c_str(),
                       pmt::symbol_to_string(tags[0].srcid).c_str());
            }
        }

        if (d_frame_end) {
            // Being here means that the previous OFDM symbol constituted the
            // end of a frame. The current symbol belongs to the next frame!
            d_frame_index = (d_frame_index + 1) % d_frames_per_superframe;
            d_symbol_index = 0;
            d_frame_sync = true;
        } else {
            d_symbol_index = (d_symbol_index + 1) % d_symbols_per_frame;
        }

        // We obtain the relative symbol index (between 0 and 3) from the block upstream
        this->get_tags_in_window(
            tags, 0, i, i + 1, pmt::string_to_symbol("relative_symbol_index"));
        if (!tags.empty()) {
            d_rel_symbol_index = pmt::to_long(tags[0].value);
        } else {
            printf("TPS decoder: No relative symbol index found in the tag stream.\n");
        }
        int diff_rel_symbol_index =
            (d_rel_symbol_index - d_prev_rel_symbol_index + 4) % 4;
        d_prev_rel_symbol_index = d_rel_symbol_index;
        if (diff_rel_symbol_index != 1) {
            if (!(d_sync_start | d_resync | !d_frame_sync)) {
                printf("TPS decoder: One or more symbols lost.\n");
            }
        }

        // Process and decode the TPS data
        d_frame_end = process_tps_data(&in[i * d_num_carriers], diff_rel_symbol_index);

        if (d_sync_start || d_resync) {
            // if (d_sync_start) {
            // if (d_resync) {
            // If sync_start or resync have been signaled, wait for the next superframe.
            // if (d_frame_sync && d_symbol_index == 0 && d_frame_index == 0 &&
            // d_tps_complete) {
            if (d_symbol_index == 0 && d_frame_index == 0 && d_tps_complete) {
                // This is a superframe start, we signal it downstream.
                d_sync_start = false;
                d_resync = false;
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
                printf("TPS decoder: Superframe start sent at offset %llu\n",
                       offset * d_num_carriers);
            } else {
                consume_each(1); // Consume the input
                return (0);      // Nothing has been produced
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

    // Tell the runtime system how many input items we have consumed on
    // each input stream.
    consume_each(noutput_items);

    // Tell the runtime system how many output items we have produced.
    return (noutput_items);
}

} /* namespace dl5eu */
} /* namespace gr */
