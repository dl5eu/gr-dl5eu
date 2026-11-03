/* -*- c++ -*- */
/*
 * Copyright 2025 Ralf Gorholt.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "dvbt_reed_solomon_dec_impl.h"
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>
#include <chrono>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace gr {
namespace dl5eu {

static const int rs_init_symsize = 8;
static const int rs_init_fcr = 0;                // first consecutive root
static const int rs_init_prim = 1;               // primitive is 1 (alpha)
static const int N = (1 << rs_init_symsize) - 1; // 255
static constexpr unsigned int max_bad_events_per_interval = 6;
static const pmt::pmt_t rs_error_tag_key = pmt::intern("rs_uncorrectable");

/*! @brief Formats the current local time for correlation with other diagnostic logs. */
static std::string rs_log_timestamp()
{
    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    std::tm local_time{};
#ifdef _WIN32
    localtime_s(&local_time, &time);
#else
    localtime_r(&time, &local_time);
#endif

    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count() % 1000;

    std::ostringstream result;
    result << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S") << '.'
           << std::setfill('0') << std::setw(3) << milliseconds;
    return result.str();
}

dvbt_reed_solomon_dec::sptr dvbt_reed_solomon_dec::make(
    int p, int m, int gfpoly, int n, int k, int t, int s, int blocks, bool diagnostics)
{
    return gnuradio::make_block_sptr<dvbt_reed_solomon_dec_impl>(
        p, m, gfpoly, n, k, t, s, blocks, diagnostics);
}

/*
 * The private constructor
 */
dvbt_reed_solomon_dec_impl::dvbt_reed_solomon_dec_impl(
    int p, int m, int gfpoly, int n, int k, int t, int s, int blocks, bool diagnostics)
    : block("dvbt_reed_solomon_dec",
            io_signature::make(1, 1, sizeof(unsigned char) * blocks * (n - s)),
            io_signature::make(1, 1, sizeof(unsigned char) * blocks * (k - s))),
      d_n(n),
      d_k(k),
      d_s(s),
      d_blocks(blocks),
      d_diagnostics_enabled(diagnostics)
{
    d_rs = init_rs_char(rs_init_symsize, gfpoly, rs_init_fcr, rs_init_prim, (n - k));
    if (d_rs == NULL) {
        this->d_logger->fatal("Reed-Solomon Decoder, cannot allocate memory for d_rs.");
        throw std::bad_alloc();
    }
    d_nerrors_corrected_count = 0;
    d_bad_packet_count = 0;
    d_total_packets = 0;
}

/*
 * Our virtual destructor.
 */
dvbt_reed_solomon_dec_impl::~dvbt_reed_solomon_dec_impl() { free_rs_char(d_rs); }

void dvbt_reed_solomon_dec_impl::forecast(int noutput_items,
                                          gr_vector_int& ninput_items_required)
{
    ninput_items_required[0] = noutput_items;
}

int dvbt_reed_solomon_dec_impl::decode(unsigned char& out, const unsigned char& in)
{
    unsigned char tmp[N];
    int ncorrections;

    // add missing prefix zero padding to message
    memset(tmp, 0, d_s);
    memcpy(&tmp[d_s], &in, (d_n - d_s));

    // correct message...
    ncorrections = decode_rs_char(d_rs, tmp, 0, 0);

    // copy corrected message to output, skipping prefix zero padding
    memcpy(&out, &tmp[d_s], (d_k - d_s));

    return ncorrections;
}

int dvbt_reed_solomon_dec_impl::general_work(int noutput_items,
                                             gr_vector_int& ninput_items,
                                             gr_vector_const_void_star& input_items,
                                             gr_vector_void_star& output_items)
{
    const unsigned char* in = (const unsigned char*)input_items[0];
    unsigned char* out = (unsigned char*)output_items[0];
    int j = 0;
    int k = 0;
    const auto output_base = nitems_written(0);

    for (int i = 0; i < (d_blocks * noutput_items); i++) {
        int nerrors_corrected = decode(out[k], in[j]);
        if (d_diagnostics_enabled) {
            ++d_interval_codewords;
        }

        if (nerrors_corrected == -1) {
            // One output item contains d_blocks RS codewords. The tag value identifies
            // the failed codeword within that item; the data remain unchanged here.
            add_item_tag(0,
                         output_base + static_cast<std::uint64_t>(i / d_blocks),
                         rs_error_tag_key,
                         pmt::from_long(i % d_blocks));

            d_bad_packet_count++;
            d_nerrors_corrected_count += ((d_n - d_s) - (d_k - d_s)) /
                                         2; // lower bound estimate; most this RS can fix

            if (d_diagnostics_enabled) {
                const auto codeword = static_cast<std::uint64_t>(d_total_packets);
                ++d_interval_uncorrectable;
                ++d_bad_burst;
                if (d_interval_uncorrectable == 1) {
                    d_interval_first_bad = codeword;
                }
                d_interval_last_bad = codeword;
                if (d_bad_burst > d_interval_max_bad_burst) {
                    d_interval_max_bad_burst = d_bad_burst;
                }

                // Limit console output during a severe RF outage.
                if (d_interval_logged_bad < max_bad_events_per_interval) {
                    std::cout << "RS_UNCORRECTABLE: " << rs_log_timestamp()
                              << " cw=" << codeword
                              << " rs_out_byte=" << codeword * (d_k - d_s)
                              << " burst=" << d_bad_burst
                              << " forwarded=1" << std::endl;
                    ++d_interval_logged_bad;
                }
            }
        } else {
            d_nerrors_corrected_count += nerrors_corrected;
            d_bad_burst = 0;
            if (d_diagnostics_enabled && nerrors_corrected > 0) {
                ++d_corrected_codewords;
                ++d_interval_corrected;
                d_corrected_symbols += nerrors_corrected;
                d_interval_corrected_symbols += nerrors_corrected;
                if (nerrors_corrected > d_interval_max_corrections) {
                    d_interval_max_corrections = nerrors_corrected;
                }
            }
        }

        d_total_packets++;
        j += (d_n - d_s);
        k += (d_k - d_s);
    }

    if (d_diagnostics_enabled) {
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - d_last_report);
        if (elapsed >= std::chrono::seconds(1)) {
            const auto clean = d_interval_codewords - d_interval_corrected -
                               d_interval_uncorrectable;
            const auto suppressed = d_interval_uncorrectable > d_interval_logged_bad
                                        ? d_interval_uncorrectable - d_interval_logged_bad
                                        : 0;

            std::cout << "RS_STAT: " << rs_log_timestamp()
                      << " interval_ms=" << elapsed.count()
                      << " cw=" << d_interval_codewords
                      << " clean=" << clean
                      << " corrected=" << d_interval_corrected
                      << " corrected_symbols=" << d_interval_corrected_symbols
                      << " max_corrections=" << d_interval_max_corrections
                      << " uncorrectable=" << d_interval_uncorrectable
                      << " total_cw=" << d_total_packets
                      << " total_corrected=" << d_corrected_codewords
                      << " total_corrected_symbols=" << d_corrected_symbols
                      << " total_uncorrectable=" << d_bad_packet_count
                      << " max_bad_burst=" << d_interval_max_bad_burst
                      << " suppressed_events=" << suppressed;
            if (d_interval_uncorrectable != 0) {
                std::cout << " first_bad_cw=" << d_interval_first_bad
                          << " last_bad_cw=" << d_interval_last_bad;
            }
            std::cout << std::endl;

            d_last_report = now;
            d_interval_codewords = 0;
            d_interval_corrected = 0;
            d_interval_corrected_symbols = 0;
            d_interval_uncorrectable = 0;
            d_interval_max_bad_burst = 0;
            d_interval_max_corrections = 0;
            d_interval_logged_bad = 0;
        }
    }

    // Tell runtime system how many input items we consumed on
    // each input stream.
    consume_each(noutput_items);

    // Tell runtime system how many output items we produced.
    return noutput_items;
}

} /* namespace dl5eu */
} /* namespace gr */
