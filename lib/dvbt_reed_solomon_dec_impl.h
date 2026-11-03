/* -*- c++ -*- */
/*
 * Copyright 2025 Ralf Gorholt.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_DL5EU_DVBT_REED_SOLOMON_DEC_IMPL_H
#define INCLUDED_DL5EU_DVBT_REED_SOLOMON_DEC_IMPL_H

#include <gnuradio/dl5eu/dvbt_reed_solomon_dec.h>
#include <chrono>
#include <cstdint>

extern "C" {
#include <gnuradio/fec/rs.h>
}

namespace gr {
namespace dl5eu {

class dvbt_reed_solomon_dec_impl : public dvbt_reed_solomon_dec
{
private:
    int d_n;
    int d_k;
    int d_s;
    int d_blocks;
    const bool d_diagnostics_enabled;

    int d_nerrors_corrected_count;
    int d_bad_packet_count;
    int d_total_packets;

    // Diagnostic counters. No changes to the decoded data or the output format.
    std::uint64_t d_corrected_codewords = 0;
    std::uint64_t d_corrected_symbols = 0;
    std::uint64_t d_interval_codewords = 0;
    std::uint64_t d_interval_corrected = 0;
    std::uint64_t d_interval_corrected_symbols = 0;
    std::uint64_t d_interval_uncorrectable = 0;
    std::uint64_t d_interval_first_bad = 0;
    std::uint64_t d_interval_last_bad = 0;
    std::uint64_t d_bad_burst = 0;
    std::uint64_t d_interval_max_bad_burst = 0;
    int d_interval_max_corrections = 0;
    unsigned int d_interval_logged_bad = 0;
    std::chrono::steady_clock::time_point d_last_report =
        std::chrono::steady_clock::now();

    void* d_rs; /* Reed-Solomon characteristics structure */
    int decode(unsigned char& out, const unsigned char& in);

public:
    dvbt_reed_solomon_dec_impl(
        int p, int m, int gfpoly, int n, int k, int t, int s, int blocks, bool diagnostics);
    ~dvbt_reed_solomon_dec_impl() override;

    void forecast(int noutput_items, gr_vector_int& ninput_items_required) override;

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items) override;
};

} // namespace dl5eu
} // namespace gr

#endif /* INCLUDED_DL5EU_DVBT_REED_SOLOMON_DEC_IMPL_H */
