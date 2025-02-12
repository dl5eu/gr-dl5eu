/* -*- c++ -*- */
/*
 * Copyright 2015,2016 Free Software Foundation, Inc.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "dvbt_convolutional_deinterleaver_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace dl5eu {

const int dvbt_convolutional_deinterleaver_impl::d_SYNC = 0x47;
const int dvbt_convolutional_deinterleaver_impl::d_NSYNC = 0xB8;
const int dvbt_convolutional_deinterleaver_impl::d_MUX_PKT = 8;

dvbt_convolutional_deinterleaver::sptr
dvbt_convolutional_deinterleaver::make(int nsize, int I, int M)
{
    return gnuradio::make_block_sptr<dvbt_convolutional_deinterleaver_impl>(nsize, I, M);
}

/*
 * The private constructor
 */
dvbt_convolutional_deinterleaver_impl::dvbt_convolutional_deinterleaver_impl(int blocks,
                                                                             int I,
                                                                             int M)
    : block("dvbt_convolutional_deinterleaver",
            io_signature::make(1, 1, sizeof(unsigned char)),
            io_signature::make(1, 1, sizeof(unsigned char) * I * blocks)),
      d_blocks(blocks),
      d_I(I),
      d_M(M)
{
    set_relative_rate(1, (uint64_t)(d_I * d_blocks));
    set_output_multiple(2);
    // The positions are shift registers (FIFOs)
    // of length i*M
    d_shift.reserve(d_I);
    for (int i = (d_I - 1); i >= 0; i--) {
        d_shift.emplace_back(d_M * i, 0);
    }

    // There are 8 mux packets
    assert(d_blocks / d_M == d_MUX_PKT);
}

/*
 * Our virtual destructor.
 */
dvbt_convolutional_deinterleaver_impl::~dvbt_convolutional_deinterleaver_impl() {}

void dvbt_convolutional_deinterleaver_impl::forecast(int noutput_items,
                                                     gr_vector_int& ninput_items_required)
{
    int ninputs = ninput_items_required.size();

    for (int i = 0; i < ninputs; i++) {
        ninput_items_required[i] = noutput_items * d_I * d_blocks;
    }
}


int dvbt_convolutional_deinterleaver_impl::general_work(
    int noutput_items,
    gr_vector_int& ninput_items,
    gr_vector_const_void_star& input_items,
    gr_vector_void_star& output_items)
{
    const unsigned char* in = (const unsigned char*)input_items[0];
    unsigned char* out = (unsigned char*)output_items[0];

    int to_out = noutput_items;

    /*
     * Look for a tag that signals superframe_start and consume all input items
     * that are in input buffer so far.
     * This will actually reset the convolutional deinterleaver
     */
    std::vector<tag_t> tags;
    const uint64_t nread = this->nitems_read(0); // number of items read on port 0
    this->get_tags_in_range(tags,
                            0,
                            nread,
                            nread + (noutput_items * d_I * d_blocks),
                            pmt::string_to_symbol("superframe_start"));

    if (!tags.empty()) {
        printf("\nConvolutional deinterleaver: %s received from %s\n",
               pmt::symbol_to_string(tags[0].key).c_str(),
               pmt::symbol_to_string(tags[0].srcid).c_str());
        printf("Convolutional deinterleaver: offset = %llu, nread = %llu\n", tags[0].offset, nread);

        if (tags[0].offset - nread) {
            printf("\nConvolutional deinterleaver: input discarded.\n");
            consume_each(tags[0].offset - nread);
            return (0);
        }
    }

    /*
     * At this moment the first item in input buffer should be NSYNC or SYNC
     */

    if (in[0] != d_NSYNC && in[0] != d_SYNC) {
        printf("\nConvolutional deinterleaver: first item in buffer: %02x.\n", in[0]);
    }

    for (int count = 0, i = 0; i < to_out; i++) {
        for (int mux_pkt = 0; mux_pkt < d_MUX_PKT; mux_pkt++) {
            // This is actually the deinterleaver
            for (int k = 0; k < (d_M * d_I); k++) {
                d_shift[k % d_I].push_back(in[count]);
                out[count++] = d_shift[k % d_I].front();
                d_shift[k % d_I].pop_front();
            }
        }
    }

    // Tell runtime system how many input items we consumed on
    // each input stream.
    consume_each(d_I * d_blocks * to_out);

    // Tell runtime system how many output items we produced.
    return (to_out);
}

} /* namespace dl5eu */
} /* namespace gr */
