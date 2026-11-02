/* -*- c++ -*- */
/*
 * Copyright 2015 Free Software Foundation, Inc.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifndef INCLUDED_DL5EU_DVBT_VITERBI_DECODER_IMPL_H
#define INCLUDED_DL5EU_DVBT_VITERBI_DECODER_IMPL_H

#include "dvbt_configure.h"
#include <gnuradio/dl5eu/dvbt_viterbi_decoder.h>
#include <chrono>
#include <cstdint>
#include <string>

#ifdef DTV_SSE2
#include <xmmintrin.h>
#endif

/* The two generator polynomials for the NASA Standard K=7 code.
 * Since these polynomials are known to be optimal for this constraint
 * length there is not much point in changing them.
 */

#define POLYA 0x4f
#define POLYB 0x6d
// Maximum number of traceback bytes
#define TRACEBACK_MAX 24

#ifdef DTV_SSE2
union branchtab27 {
    unsigned char c[32];
    __m128i v[2];
};
#else
struct branchtab27 {
    unsigned char c[32];
};
#endif

namespace gr {
namespace dl5eu {

/*!
 * @brief Decode hard-decision DVB-T convolutional data with puncturing support.
 *
 * Input bytes hold one demapped constellation label each (d_m meaningful bits).
 * Depuncturing inserts erasures, then the K=7, 64-state trellis produces decoded
 * bytes. SSE2 and generic paths implement the same hard-decision algorithm.
 * resync tags reset trellis acquisition; frame tags are mapped into byte offsets
 * with traceback delay accounted for. Static trellis storage is shared between
 * instances, so it must not be assumed to be independent per block.
 */
class dvbt_viterbi_decoder_impl : public dvbt_viterbi_decoder
{
private:
    dvbt_configure config;

    // Puncturing vectors
    static const unsigned char d_puncture_1_2[];
    static const unsigned char d_puncture_2_3[];
    static const unsigned char d_puncture_3_4[];
    static const unsigned char d_puncture_5_6[];
    static const unsigned char d_puncture_7_8[];
    static const unsigned char d_Partab[];

#ifdef DTV_SSE2
    static __m128i d_metric0[4];
    static __m128i d_metric1[4];
    static __m128i d_path0[4];
    static __m128i d_path1[4];
#else
    static unsigned char d_metric0_generic[64];
    static unsigned char d_metric1_generic[64];
    static unsigned char d_path0_generic[64];
    static unsigned char d_path1_generic[64];
#endif

#ifdef DTV_SSE2
    static branchtab27 Branchtab27_sse2[2];
#else
    static branchtab27 Branchtab27_generic[2];
#endif

    // Metrics for each state
    static unsigned char mmresult[64];
    // Paths for each state
    static unsigned char ppresult[TRACEBACK_MAX][64];

    // Current puncturing vector
    const unsigned char* d_puncture;

    // Code rate k/n
    const int d_k;
    const int d_n;
    // Number of coded bits represented by each input constellation label.
    const int d_m;

    // Block size
    const int d_bsize;
    // Symbols to consume on decoding from one block.
    // It is also the number of input bytes since
    // one byte always contains just one symbol.
    const int d_nsymbols;
    // Number of bits after depuncturing a block (before decoding)
    const int d_nbits;

    // Traceback (in bytes)
    int d_ntraceback;

    // Viterbi tables
    int mettab[2][256];

    // Depunctured mother-code decisions: 0/1 for received bits, 2 for erasures.
    std::vector<unsigned char> d_inbits;

    // Startup/restart gate for suppressing unreliable initial traceback bytes.
    int d_init = 0;

    // Position in circular buffer where the current decoded byte is stored
    int store_pos = 0;

    const bool d_diagnostics;

    // Diagnostic state (no effect on decoding or tag propagation).
    using diagnostic_clock = std::chrono::steady_clock;
    diagnostic_clock::time_point d_diag_last_report = diagnostic_clock::now();
    uint64_t d_diag_input_items = 0;
    uint64_t d_diag_output_items = 0;
    uint64_t d_diag_prev_input_items = 0;
    uint64_t d_diag_prev_output_items = 0;
    uint64_t d_diag_resync_tags = 0;
    uint64_t d_diag_resets = 0;
    uint64_t d_diag_restarts = 0;
    uint64_t d_diag_frame_tags_in = 0;
    uint64_t d_diag_frame_tags_out = 0;
    uint64_t d_diag_last_frame_input = 0;
    uint64_t d_diag_last_frame_output = 0;
    uint64_t d_diag_tracebacks = 0;
    uint64_t d_diag_metric_samples = 0;
    uint64_t d_diag_best_sum = 0;
    uint64_t d_diag_spread_sum = 0;
    unsigned int d_diag_spread_min = 255;
    unsigned int d_diag_spread_max = 0;
    int d_diag_last_minmetric = 0;
    unsigned int d_diag_logged_events = 0;
    uint64_t d_diag_suppressed_events = 0;
    bool d_diag_first_frame_after_restart = false;

    /*!
     * @brief Emit periodic rate, reset, tag and sampled path-metric statistics.
     *
     * Uses steady time for intervals. Metric spread is a diagnostic indicator, not a
     * calibrated BER or an error-correction count; diagnostics do not alter decisions.
     */
    void report_diagnostics();
    /*!
     * @brief Emit a rate-limited timestamped decoder event when diagnostics are enabled.
     * @param type Event label.
     * @param details Context such as absolute input/output offsets.
     *
     * Suppressed-event totals preserve evidence of event bursts without console flooding.
     */
    void diagnostic_event(const char* type, const std::string& details);

#ifdef DTV_SSE2
    /*!
     * @brief Initialize equal starting metrics, survivor histories and branch tables.
     * @param mm0 Metric storage for all 64 states (packed in the SSE2 variant).
     * @param pp0 Survivor storage for all 64 states.
     *
     * All states start equally; no known encoder state is imposed at acquisition.
     */
    void dvbt_viterbi_chunks_init_sse2(__m128i* mm0, __m128i* pp0);
    /*!
     * @brief Advance the 64-state trellis by two decoded bits.
     * @param symbols Four mother-code symbols: 0/1 are hard decisions, 2 is an erasure.
     * @param m0 Metric bank before and after the two trellis steps.
     * @param m1 Intermediate metric bank.
     * @param p0 Survivor bank before and after both steps.
     * @param p1 Intermediate survivor bank.
     *
     * Select the better predecessor for each state; punctured erasures contribute no
     * preference for either bit value.
     */
    void dvbt_viterbi_butterfly2_sse2(
        unsigned char* symbols, __m128i m0[], __m128i m1[], __m128i p0[], __m128i p1[]);
    /*!
     * @brief Trace back from the best state and recover one delayed decoded byte.
     * @param mm0 Current metrics for 64 states.
     * @param pp0 Current packed survivor histories.
     * @param ntraceback Circular-history depth in bytes, at most TRACEBACK_MAX.
     * @param outbuf Receives the recovered byte; initial traceback output is discarded.
     * @return Best path metric before normalization, not a decoded byte or BER.
     *
     * Save survivors in the circular history, follow predecessors and subtract the
     * minimum metric to prevent metric growth without changing relative scores.
     */
    unsigned char dvbt_viterbi_get_output_sse2(__m128i* mm0,
                                               __m128i* pp0,
                                               int ntraceback,
                                               unsigned char* outbuf);
#else
    /*!
     * @brief Initialize equal starting metrics, survivor histories and branch tables.
     * @param mm0 Metric storage for all 64 states (packed in the SSE2 variant).
     * @param pp0 Survivor storage for all 64 states.
     *
     * All states start equally; no known encoder state is imposed at acquisition.
     */
    void dvbt_viterbi_chunks_init_generic(unsigned char* mm0, unsigned char* pp0);
    /*!
     * @brief Advance the 64-state trellis by two decoded bits.
     * @param symbols Four mother-code symbols: 0/1 are hard decisions, 2 is an erasure.
     * @param m0 Metric bank before and after the two trellis steps.
     * @param m1 Intermediate metric bank.
     * @param p0 Survivor bank before and after both steps.
     * @param p1 Intermediate survivor bank.
     *
     * Select the better predecessor for each state; punctured erasures contribute no
     * preference for either bit value.
     */
    void dvbt_viterbi_butterfly2_generic(unsigned char* symbols,
                                         unsigned char m0[],
                                         unsigned char m1[],
                                         unsigned char p0[],
                                         unsigned char p1[]);
    /*!
     * @brief Trace back from the best state and recover one delayed decoded byte.
     * @param mm0 Current metrics for 64 states.
     * @param pp0 Current packed survivor histories.
     * @param ntraceback Circular-history depth in bytes, at most TRACEBACK_MAX.
     * @param outbuf Receives the recovered byte; initial traceback output is discarded.
     * @return Best path metric before normalization, not a decoded byte or BER.
     *
     * Save survivors in the circular history, follow predecessors and subtract the
     * minimum metric to prevent metric growth without changing relative scores.
     */
    unsigned char dvbt_viterbi_get_output_generic(unsigned char* mm0,
                                                  unsigned char* pp0,
                                                  int ntraceback,
                                                  unsigned char* outbuf);
#endif

public:
    /*!
     * @brief Select puncturing, traceback depth and scheduling granularity.
     * @param constellation Demapper constellation; determines meaningful bits per byte.
     * @param hierarchy Hierarchical modulation configuration used by dvbt_configure.
     * @param coderate Convolutional code rate selecting the puncturing pattern.
     * @param bsize Number of puncturing periods processed per decoding block.
     * @param diagnostics Enable bounded event and periodic metric reporting.
     *
     * Input block size is bsize*n/m labels; output size is bsize*k/8 bytes for rate
     * k/n. Block dimensions must satisfy the packing and output-multiple constraints.
     */
    dvbt_viterbi_decoder_impl(dvb_constellation_t constellation,
                              dvbt_hierarchy_t hierarchy,
                              dvb_code_rate_t coderate,
                              int bsize,
                              bool diagnostics);
    ~dvbt_viterbi_decoder_impl() override;

    /*!
     * @brief Convert the requested decoded-byte count to demapped input labels.
     * @param noutput_items Requested decoded bytes.
     * @param ninput_items_required Receives input counts using constellation and code rate.
     */
    void forecast(int noutput_items, gr_vector_int& ninput_items_required) override;

    /*!
     * @brief Unpack labels, insert punctured erasures and decode complete trellis blocks.
     * @param noutput_items Output byte capacity, aligned to the configured block multiple.
     * @param ninput_items Available input item counts supplied by the scheduler.
     * @param input_items Hard-decision constellation-label buffers.
     * @param output_items Decoded-byte buffers.
     * @return Bytes produced after removing initial traceback; consumption is separate.
     *
     * Discard input preceding a resync tag and reset when that tag reaches the next
     * input origin. The restart discards unreliable traceback bytes and emits a new
     * superframe_start. Explicit frame_start mapping replaces automatic propagation.
     */
    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items) override;
};

} // namespace dl5eu
} // namespace gr

#endif /* INCLUDED_DL5EU_DVBT_VITERBI_DECODER_IMPL_H */
