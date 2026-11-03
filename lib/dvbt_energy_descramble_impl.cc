/* -*- c++ -*- */
/*
 * Copyright 2025 Ralf Gorholt.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "dvbt_energy_descramble_impl.h"
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <vector>

namespace gr {
namespace dl5eu {

const int dvbt_energy_descramble_impl::d_nblocks = 8;
const int dvbt_energy_descramble_impl::d_bsize = 188;
const int dvbt_energy_descramble_impl::d_SYNC = 0x47;
const int dvbt_energy_descramble_impl::d_NSYNC = 0xB8;
const int dvbt_energy_descramble_impl::d_MUX_PKT = 8;

void dvbt_energy_descramble_impl::init_prbs() { d_reg = 0xa9; }

int dvbt_energy_descramble_impl::clock_prbs(int clocks)
{
    int res = 0;
    int feedback = 0;

    for (int i = 0; i < clocks; i++) {
        feedback = ((d_reg >> (14 - 1)) ^ (d_reg >> (15 - 1))) & 0x1;
        d_reg = ((d_reg << 1) | feedback) & 0x7fff;

        res = (res << 1) | feedback;
    }

    return res;
}

dvbt_energy_descramble::sptr dvbt_energy_descramble::make(int nblocks)
{
    return gnuradio::make_block_sptr<dvbt_energy_descramble_impl>(nblocks);
}

/*
 * The private constructor
 */
dvbt_energy_descramble_impl::dvbt_energy_descramble_impl(int nblocks)
    : block("dvbt_energy_descramble",
            io_signature::make(1, 1, sizeof(unsigned char) * d_nblocks * d_bsize),
            io_signature::make(1, 1, sizeof(unsigned char))),
      d_reg(0xa9),
      d_index(0),
      d_search(0)
{
    set_relative_rate((uint64_t)(d_nblocks * d_bsize), 1);

    // RS error tags identify codewords within input items, not output bytes.
    set_tag_propagation_policy(TPP_DONT);

    // Set output multiple of (2 search + 1 data = 3) at least (4 for now)
    set_output_multiple(4 * d_nblocks * d_bsize);

    // Search interval for NSYNC is 2 * 8 * MUX size
    d_search = 2 * (d_nblocks * d_bsize);
}

/*
 * Our virtual destructor.
 */
dvbt_energy_descramble_impl::~dvbt_energy_descramble_impl() {}

void dvbt_energy_descramble_impl::forecast(int noutput_items,
                                           gr_vector_int& ninput_items_required)
{
    ninput_items_required[0] = 4 * (noutput_items / (d_nblocks * d_bsize));
}

int dvbt_energy_descramble_impl::general_work(int noutput_items,
                                              gr_vector_int& ninput_items,
                                              gr_vector_const_void_star& input_items,
                                              gr_vector_void_star& output_items)
{
    const unsigned char* in = (const unsigned char*)input_items[0];
    unsigned char* out = (unsigned char*)output_items[0];

    int to_consume, to_out;

    // Search for a NSYNC
    while ((in[d_index] != d_NSYNC) && (d_index < d_search)) {
        d_index += d_bsize;
    }

    /*
     * If we did not find a NSYNC then just consume
     * and return 0.
     */
    if (d_index >= d_search) {
        d_index = 0;
        to_consume = 2;
        to_out = 0;
    } else {
        // We found a NSYNC, descramble the data

        to_consume = (noutput_items / (d_nblocks * d_bsize)) - 2;
        to_out = noutput_items - 2 * (d_nblocks * d_bsize);

        // The input stream consists of items containing eight 188-byte RS codewords.
        // Each rs_uncorrectable tag belongs to an input item; its value (0..7)
        // identifies the affected codeword within that item. d_index can skip
        // packets during synchronization search, so use absolute input offsets.
        const int item_size = d_nblocks * d_bsize;
        const int first_item = d_index / item_size;
        const int end_item = (d_index + to_out + item_size - 1) / item_size;
        const std::uint64_t input_base = nitems_read(0);
        const std::uint64_t first_tag_offset = input_base + first_item;

        std::vector<tag_t> rs_error_tags;
        get_tags_in_range(rs_error_tags,
                          0,
                          first_tag_offset,
                          input_base + end_item,
                          pmt::intern("rs_uncorrectable"));

        std::vector<std::uint8_t> rs_error_masks(end_item - first_item, 0);
        for (const auto& tag : rs_error_tags) {
            if (!pmt::is_integer(tag.value)) {
                continue;
            }

            const long codeword = pmt::to_long(tag.value);
            if (codeword >= 0 && codeword < d_nblocks) {
                const auto item = static_cast<std::size_t>(tag.offset - first_tag_offset);
                rs_error_masks[item] |= static_cast<std::uint8_t>(1U << codeword);
            }
        }

        for (int count = 0, i = 0; i < to_consume; i++) {
            init_prbs();

            for (int mux_pkt = 0; mux_pkt < d_MUX_PKT; mux_pkt++) {
                const int packet_start = count;
                const int input_packet_start = d_index + packet_start;
                const int input_item = input_packet_start / item_size - first_item;
                const int codeword = (input_packet_start % item_size) / d_bsize;
                const bool uncorrectable =
                    (rs_error_masks[input_item] & (1U << codeword)) != 0;

                out[count++] = d_SYNC;
                // PRBS clocking starts right after NSYNC

                for (int k = 1; k < d_bsize; k++) {
                    out[count] = in[d_index + count] ^ clock_prbs(d_nblocks);
                    count++;
                }

                // The TS header is now descrambled; mark uncorrectable RS data.
                if (uncorrectable) {
                    out[packet_start + 1] |= 0x80;
                }

                // For subsequent blocks PRBS is clocked also on SYNC
                // but its output is not used
                clock_prbs(d_nblocks);
            }
        }
    }

    // Tell runtime how many input items we consumed
    consume_each(to_consume);

    // Tell runtime system how many output items we produced.
    return (to_out);
}

} /* namespace dl5eu */
} /* namespace gr */
