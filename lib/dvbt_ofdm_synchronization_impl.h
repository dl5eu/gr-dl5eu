/* -*- c++ -*- */
/*
 * Copyright 2025 Ralf Gorholt.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * ==============================================================
 *  This block is based on the block "ofdm_synchronization" that
 *  Federico La Rocca and others have developed for ISDB-T.
 * ==============================================================
 */

#ifndef INCLUDED_DL5EU_DVBT_OFDM_SYNCHRONIZATION_IMPL_H
#define INCLUDED_DL5EU_DVBT_OFDM_SYNCHRONIZATION_IMPL_H

#include "dvbt_configure.h"
#include "dvbt_pilot_gen.h"
#include <gnuradio/dl5eu/dvbt_ofdm_synchronization.h>
#include <gnuradio/fft/fft.h>
#include <gnuradio/filter/mmse_fir_interpolator_cc.h>
#include <atomic>
#include <cstdint>
#include <cstddef>
#include <array>

namespace gr {
namespace dl5eu {

/*!
 * @brief Acquire DVB-T OFDM symbols and equalize their active carriers.
 *
 * CP correlation provides symbol timing and fractional carrier-frequency acquisition.
 * Continual and scattered pilots provide integer frequency acquisition, channel
 * estimates and fine frequency/clock tracking. Output port 0 contains one vector of
 * active carriers per symbol; optional ports expose channel gain, frequency error
 * in Hz and sampling-rate error in ppm. Tag offsets on these outputs count symbols,
 * whereas input offsets count complex time-domain samples.
 */
/*!
 * @brief Acquire DVB-T OFDM symbols and equalize their active carriers.
 *
 * CP correlation provides symbol timing and fractional carrier-frequency acquisition.
 * Continual and scattered pilots provide integer frequency acquisition, channel
 * estimates and fine frequency/clock tracking. Output port 0 contains one vector of
 * active carriers per symbol; optional ports expose channel gain, frequency error
 * in Hz and sampling-rate error in ppm. Tag offsets on these outputs count symbols,
 * whereas input offsets count complex time-domain samples.
 */
class dvbt_ofdm_synchronization_impl : public dvbt_ofdm_synchronization
{
    // The configuration object of this class.
    // Should be first to be initialized first.
    const dvbt_configure d_config;

private:
    dvbt_pilot_gen d_pg;

    // Variables to keep data for 2K and 8K transmission mode
    const int d_num_carriers;
    const int d_num_cpilots;
    const int d_num_spilots;
    const int* d_cpilots;

    const int d_fft_length;
    const int d_cp_length;
    std::atomic<double> d_sample_rate;
    std::atomic<bool> d_resynchronization_requested{ false };
    uint64_t d_reconfiguration_discard_remaining{ 0 };
    std::atomic<float> d_squelch_db;
    const bool d_diagnostics;

    // For symbol time and coarse frequency synchronization
    int d_cp_start;
    bool d_cp_found;
    volk::vector<gr_complex> d_gamma;
    volk::vector<float> d_phi;
    volk::vector<float> d_lambda;
    float d_coarse_freq;
    volk::vector<gr_complex> d_derot;
    volk::vector<gr_complex> d_conj;
    volk::vector<float> d_norm;
    volk::vector<gr_complex> d_corr;

    // ml_sync
    const float d_snr;
    const float d_rho;

    // Peak detection
    float d_threshold_factor_rise;
    float d_avg_alpha;
    float d_avg_min;
    float d_avg_max;
    float d_cp_quality{ 0.0f };

    // Diagnostics only: reason and raw ML metrics of the last CP-search rejection.
    struct cp_rejection_diagnostic {
        const char* reason{ "none" };
        int peak_index{ -1 };
        int search_size{ 0 };
        float peak_metric{ 0.0f };
        float threshold{ 0.0f };
        float quality{ 0.0f };
    };
    cp_rejection_diagnostic d_diag_cp_rejection;

    // Phase correction
    double d_phaseinc;
    double d_nextphaseinc;
    int d_nextpos;
    float d_phase;

    bool d_initial_acquired;

    // Tentative acquisition: number of consistent CP peaks and expected position.
    int d_acquisition_matches{ 0 };
    int d_acquisition_cp_position{ 0 };

    // Diagnostics for input-signal transitions and CP acquisition.
    bool d_input_power_initialized{ false };
    bool d_input_signal_present{ false };
    bool d_signal_return_seen{ false };
    uint64_t d_signal_return_sample{ 0 };
    uint64_t d_diag_full_searches{ 0 };
    uint64_t d_diag_first_candidates{ 0 };
    uint64_t d_diag_confirmation_failures{ 0 };
    uint64_t d_diag_peak_overrides{ 0 };

    // Record the CP state immediately before a loss and check phase on reacquisition.
    bool d_diag_last_good_valid{ false };
    bool d_diag_await_reacquisition{ false };
    uint64_t d_diag_last_good_cp_sample{ 0 };
    float d_diag_last_good_quality{ 0.0f };
    double d_diag_last_good_power_db{ 0.0 };
    std::array<float, 16> d_diag_recent_qualities{};
    std::size_t d_diag_recent_count{ 0 };
    std::size_t d_diag_recent_next{ 0 };

    int d_consumed;
    int d_to_output;

    // FFT part
    volk::vector<gr_complex> d_postfft;
    gr::fft::fft_complex_fwd d_fft_calculator;

    // Inactive FFT bins preceding the nominal active-carrier range.
    int d_zeros_on_left;
    // Integer frequency variables
    int d_freq_offset_max;
    int d_freq_offset;
    int d_freq_offset_agree_count;
    bool d_freq_offset_acq;

    volk::vector<float> d_known_phase_diff;
    gr_complex* d_integer_freq_derotated;

    // Symbol estimation and channel equalization part
    volk::vector<gr_complex> d_pilot_values;
    volk::vector<gr_complex> d_channel_gain;
    // Cached linear weights and scratch buffers for the 11 bins between pilots.
    volk::vector<gr_complex> d_coeffs_linear_estimate_first;
    volk::vector<gr_complex> d_aux_linear_estimate_first;
    volk::vector<gr_complex> d_coeffs_linear_estimate_last;
    volk::vector<gr_complex> d_aux_linear_estimate_last;
    volk::vector<float> d_channel_gain_mag_sq;
    volk::vector<float> d_ones;
    volk::vector<gr_complex> d_channel_gain_inv;

    int d_current_symbol;
    int d_previous_symbol;
    bool d_symbol_acq;
    int d_symbol_correct_count;
    volk::vector<float> d_corr_sp;

    // Independent scattered-pilot observation; never changes the tracked symbol index.
    uint64_t d_diag_sp_checks{ 0 };
    uint64_t d_diag_sp_mismatches{ 0 };
    uint64_t d_diag_sp_mismatch_run{ 0 };

    // Fine frequency and symbol synchronization
    volk::vector<gr_complex> d_previous_channel_gain;
    volk::vector<gr_complex> d_delta_channel_gains;
    // Whether to use sampling correction (may be costly)
    bool d_interpolate;
    volk::vector<gr_complex> d_interpolated;
    gr::filter::mmse_fir_interpolator_cc d_inter;
    float d_samp_inc;
    float d_samp_phase;
    int d_cp_start_offset;

    float d_alpha_freq;
    float d_beta_freq;
    float d_alpha_timing;
    float d_beta_timing;
    float d_freq_aux;
    float d_fine_freq;
    float d_delta_aux;
    float d_est_delta;
    bool d_cp_moved;

    // The total frequency error (used for derotation)
    float d_total_freq_error;

    /*!
     * @brief Update the second-order residual carrier-frequency tracking loop.
     * @param error Pilot-derived phase error in radians, normalized to the useful symbol.
     *
     * The integral state tracks persistent error; the proportional term responds to
     * new measurements. The resulting estimate contributes to the next derotation.
     */
    void advance_freq_loop(float error);

    /*!
     * @brief Update the second-order sampling-clock tracking loop.
     * @param error Dimensionless relative sampling-rate error from pilot phase slope.
     *
     * Accumulated drift adjusts the fractional input step used by the interpolator.
     */
    void advance_delta_loop(float error);

    /*!
     * @brief Resample one complete symbol, including its guard interval.
     * @param in Input samples, including the MMSE interpolator's required look-ahead.
     * @param out Storage for FFT length plus CP length complex samples.
     * @return Number of input samples advanced, excluding interpolation look-ahead.
     *
     * Fractional phase persists across calls so clock correction remains continuous.
     */
    int interpolate_input(const gr_complex* in, gr_complex* out);

    /*!
     * @brief Derive residual frequency and clock errors from successive channel estimates.
     * @param current_channel Current symbol's complex gains for all active carriers.
     * @param previous_channel Previous symbol's gains in the same carrier order.
     *
     * Correlations over the lower and upper carrier halves separate common phase
     * rotation from frequency-dependent phase slope. A moved FFT window invalidates
     * this comparison, so loop updates are suppressed for that transition.
     */
    void estimate_fine_synchro(gr_complex* current_channel, gr_complex* previous_channel);

    /*!
     * @brief Emit pilot phase and flag discontinuities in its modulo-four sequence.
     * @param current_offset Output symbol offset relative to nitems_written(0).
     *
     * The relative_symbol_index tag carries phase 0..3. Unexpected progression emits
     * resync; more than three consecutive correct transitions establishes phase lock.
     */
    void send_symbol_index_and_resync(int current_offset);

    /*!
     * @brief Fill channel gains between the current symbol's pilot measurements.
     * @param current_symbol Scattered-pilot phase, 0..3.
     * @param channel_gain In/out array of gains for all active carriers.
     *
     * Interpolate between scattered pilots, extrapolate below the first one and use
     * the final continual pilot at the upper edge. No averaging across symbols is used,
     * so a transient phase change is not spread into later channel estimates.
     */
    void linearly_estimate_channel_taps(int current_symbol, gr_complex* channel_gain);

    /*!
     * @brief Measure channel gain at scattered pilots and the final continual pilot.
     * @param in Active carriers after integer frequency alignment, before equalization.
     * @param current_symbol Scattered-pilot phase, 0..3.
     * @param channel_gain Output array; only measured pilot positions are written.
     *
     * Divide each received pilot by its known transmitted value. Interpolation fills
     * the remaining positions in a separate step.
     */
    void calculate_channel_taps_sp(const gr_complex* in,
                                   int current_symbol,
                                   gr_complex* channel_gain);

    /*!
     * @brief Score the four scattered-pilot patterns using adjacent pilot correlations.
     * @param in Integer-aligned active carriers before equalization.
     * @return Phase 0..3 with the largest correlation magnitude.
     *
     * Also refreshes d_corr_sp for independent phase diagnostics. Calling this method
     * does not itself change the tracked symbol index.
     */
    int estimate_symbol_index(const gr_complex* in);

    /*!
     * @brief Cache the pilot generator's known PRBS-derived carrier values.
     *
     * These reference amplitudes and polarities support pilot correlation and channel
     * gain estimation; no received signal is required.
     */
    void generate_prbs();

    /*!
     * @brief Search continual-pilot polarity transitions for an integer carrier shift.
     * @param in FFT-shifted vector containing the complete FFT, including inactive bins.
     * @return Residual frequency shift in FFT bins relative to nominal active carriers.
     *
     * Search around the nominal band and require repeated zero residual estimates
     * before stopping acquisition. The accumulated shift is subsequently derotated.
     */
    int estimate_integer_freq_offset(const gr_complex* in);

    /*!
     * @brief Transform the FFT object's input buffer and center the spectrum.
     * @param out Storage for d_fft_length bins, ordered with negative frequencies first.
     *
     * The caller supplies an already timed and derotated useful-symbol window.
     */
    void calculate_fft(gr_complex* out);

    /*!
     * @brief Find a credible CP-correlation peak within a bounded search interval.
     * @param in Time-domain samples covering both FFT-separated correlation windows.
     * @param lookup_start Exclusive upper candidate index (clamped to buffer capacity).
     * @param lookup_stop Inclusive lower candidate index.
     * @param cp_pos Receives the last sample index of the later correlation window.
     * @param peak_epsilon Receives the correlation phase in radians for coarse derotation.
     * @return True when quality and peak-position checks accept the candidate.
     *
     * Sliding correlation and energy sums avoid re-summing each CP. The selected
     * maximum-likelihood peak must have normalized correlation at least 0.30.
     * During established tracking only, a strong nearby peak may bypass an obsolete
     * adaptive amplitude threshold; border and low-quality rejections remain effective.
     * The cp_pos convention is a correlation-window end, not the first guard sample.
     */
    bool ml_sync(const gr_complex* in,
                 int lookup_start,
                 int lookup_stop,
                 int* cp_pos,
                 float* peak_epsilon);

    /*!
     * @brief Reset adaptive peak levels so acquisition learns the current signal level.
     * @param threshold_factor_rise Fraction of the peak-to-background span tolerated
     * below the smoothed maximum.
     * @param alpha Weight of a new measurement in the exponential level averages.
     *
     * Infinite sentinels ensure the first observation initializes each average directly.
     */
    void init_peak_detection(float threshold_factor_rise, float alpha);

    /*!
     * @brief Locate the ML maximum and apply border and adaptive-level checks.
     * @param data Search metrics for successive candidate positions.
     * @param data_size Number of candidate positions; must be positive.
     * @param peak_pos Receives the maximum's array index, also on rejection.
     * @return True if the candidate passes both checks.
     *
     * Full searches update the background estimate; narrow tracking searches reuse it.
     * The rejection reason allows ml_sync() to distinguish stale amplitude thresholds
     * from peaks whose timing is unreliable.
     */
    bool detect_peak_position(const float* data, const int data_size, int* peak_pos);

    /*!
     * @brief Mark the output position at which downstream frame acquisition must restart.
     * @param current_offset Symbol offset relative to nitems_written(0).
     *
     * The tag can target the next output item when loss is detected before that item
     * is produced. It does not assert that TPS frame synchronization is already known.
     */
    void send_sync_start(int current_offset);

    /*!
     * @brief Remove accumulated coarse, fine and integer carrier-frequency error.
     * @param in Useful-symbol samples, with timing already selected.
     * @param out Storage for d_fft_length derotated samples.
     *
     * Preserve oscillator phase between symbols and advance it across the skipped CP
     * to avoid introducing an artificial phase discontinuity at each FFT window.
     */
    void derotate(const gr_complex* in, gr_complex* out);
    
    /*!
     * @brief Clear acquisition and tracking loops after signal or synchronization loss.
     *
     * Reset frequency integrators, clock correction and adaptive peak levels so stale
     * estimates do not delay reacquisition. Diagnostic history and configured settings
     * are retained; the caller is responsible for notifying downstream blocks.
     */
     void reset_synchronization();

public:
    /*!
     * @brief Configure OFDM dimensions, pilot references and tracking-loop coefficients.
     * @param transmission_mode DVB-T FFT mode (2K or 8K).
     * @param guard_interval Configured cyclic-prefix fraction.
     * @param snr Assumed SNR in dB for ML weighting; not a measured output SNR.
     * @param interpolate Enable MMSE sampling-clock correction.
     * @param sample_rate Input complex-sample rate in samples/s, used for Hz conversion.
     * @param squelch_db Digital IQ opening threshold in dBFS; values <= -200 disable it.
     * @param diagnostics Enable extended CP-loss probes, not all event messages.
     * @throws std::invalid_argument If sample rate is not positive/finite or the
     * squelch threshold is not finite.
     */
    dvbt_ofdm_synchronization_impl(dvbt_transmission_mode_t transmission_mode,
                                   dvb_guardinterval_t guard_interval,
                                   float snr,
                                   bool interpolate,
                                   double sample_rate,
                                   float squelch_db,
                                   bool diagnostics);
    ~dvbt_ofdm_synchronization_impl();

    void set_sample_rate(double sample_rate) override;
    double get_sample_rate() const override;
    void request_resynchronization() override;

    /*!
     * @brief Atomically change the digital-IQ squelch opening threshold.
     * @param threshold_db Threshold in dBFS; values <= -200 disable squelch.
     * @throws std::invalid_argument If threshold_db is not finite.
     *
     * The processing thread reads this setting without resetting a running flowgraph.
     * The closing threshold is 3 dB below the opening threshold.
     */
    void set_squelch_threshold(float threshold_db) override;
    /*!
     * @brief Read the current runtime squelch setting.
     * @return Opening threshold in dBFS, including the disabling value if configured.
     */
    float get_squelch_threshold() const override;

    /*!
     * @brief Request time-domain input for the requested number of output symbols.
     * @param noutput_items Requested output vector count.
     * @param ninput_items_required Receives input sample requirements per port.
     *
     * Allow an extra symbol for acquisition searches and MMSE look-ahead; account for
     * the current fractional sample step.
     */
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    /*!
     * @brief Acquire, track, FFT and equalize complete OFDM symbols.
     * @param noutput_items Capacity of each output in symbol items.
     * @param ninput_items Available complex input samples per input port.
     * @param input_items Time-domain IQ buffers.
     * @param output_items Active-carrier vectors and optional diagnostic buffers.
     * @return Number of output symbols produced; input consumption is reported separately.
     *
     * A closed squelch consumes noise without output. Acquisition requires three
     * consistent CP peaks. Tracking uses narrow searches, then a wider retry; loss
     * resets the loops and signals downstream. Independent pilot-phase diagnostics
     * do not change the established modulo-four phase. Early returns preserve the
     * actual sample consumption when acquisition or window recentering shifts input.
     */
    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);
};

} // namespace dl5eu
} // namespace gr

#endif /* INCLUDED_DL5EU_DVBT_OFDM_SYNCHRONIZATION_IMPL_H */
