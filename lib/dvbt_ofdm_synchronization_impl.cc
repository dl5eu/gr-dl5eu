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

#include "dvbt_ofdm_synchronization_impl.h"
#include <gnuradio/io_signature.h>

#include <cstdio>
#include <cinttypes>
#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <limits>
#include <stdexcept>
#include <string>

namespace gr {
namespace dl5eu {

static const char* blockid = "dvbt_ofdm_synchronization";

namespace {
/*!
 * @brief Convert the assumed SNR in dB into the CP likelihood weighting S/(S+N).
 * @param snr Assumed signal-to-noise ratio in dB.
 * @return Dimensionless weighting used in the ML energy penalty.
 */
float calculate_rho(float snr)
{
    snr = pow(10, snr / 10.0);
    return snr / (snr + 1.0);
}
/*!
 * @brief Return the active-carrier vector length for the configured FFT mode.
 * @param transmission_mode 2K or 8K mode; the fallback follows the 2K path.
 * @return 1705 or 6817 complex carriers per output item.
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

// CP loss diagnostic only. No synchronization, peak-detector, or receiver state is changed.
struct cp_probe_result
{
    int positions{ 0 };
    int full_positions{ 0 };
    int best_quality_pos{ -1 };
    int best_metric_pos{ -1 };
    int best_8_pos{ -1 };
    int best_16_pos{ -1 };
    double best_quality{ 0.0 };
    double metric_peak_quality{ 0.0 };
    double best_8_quality{ 0.0 };
    double best_16_quality{ 0.0 };
    double expected_quality{ -1.0 };
    double expected_early_power{ 0.0 };
    double expected_late_power{ 0.0 };
};

/*! \brief Examines a complete CP search interval without affecting receiver state. */
cp_probe_result probe_cp_correlation(const gr_complex* input,
                                     int available,
                                     int fft_length,
                                     int cp_length,
                                     int expected,
                                     double rho)
{
    cp_probe_result result;
    const int first_position = fft_length + cp_length - 1;
    const int last_position = 2 * fft_length + cp_length - 2;
    result.full_positions = last_position - first_position + 1;
    const int end_position = std::min(last_position, available - 1);
    if (end_position < first_position) {
        return result;
    }

    auto product = [input](int late, int early) -> std::complex<double> {
        const gr_complex value = input[late] * std::conj(input[early]);
        return { value.real(), value.imag() };
    };

    int early = first_position - cp_length + 1 - fft_length;
    int late = early + fft_length;
    std::complex<double> cross{};
    double energy_early = 0.0;
    double energy_late = 0.0;
    for (int j = 0; j < cp_length; ++j) {
        cross += product(late + j, early + j);
        energy_early += std::norm(input[early + j]);
        energy_late += std::norm(input[late + j]);
    }

    double best_metric = -std::numeric_limits<double>::infinity();
    for (int position = first_position; position <= end_position; ++position) {
        const double mean_energy = (energy_early + energy_late) / 2.0;
        const double correlation = std::abs(cross);
        const double quality = mean_energy > 1.0e-20 ? correlation / mean_energy : 0.0;
        const double metric = correlation - rho * mean_energy;

        if (std::isfinite(quality) && std::isfinite(metric)) {
            if (quality > result.best_quality || result.best_quality_pos == -1) {
                result.best_quality = quality;
                result.best_quality_pos = position;
            }
            if (metric > best_metric) {
                best_metric = metric;
                result.metric_peak_quality = quality;
                result.best_metric_pos = position;
            }
            if (std::abs(position - expected) <= 8 &&
                (quality > result.best_8_quality || result.best_8_pos == -1)) {
                result.best_8_quality = quality;
                result.best_8_pos = position;
            }
            if (std::abs(position - expected) <= 16 &&
                (quality > result.best_16_quality || result.best_16_pos == -1)) {
                result.best_16_quality = quality;
                result.best_16_pos = position;
            }
            if (position == expected) {
                result.expected_quality = quality;
                result.expected_early_power = energy_early / cp_length;
                result.expected_late_power = energy_late / cp_length;
            }
        }
        ++result.positions;

        if (position < end_position) {
            cross += product(late + cp_length, early + cp_length) -
                     product(late, early);
            energy_early += std::norm(input[early + cp_length]) -
                            std::norm(input[early]);
            energy_late += std::norm(input[late + cp_length]) -
                           std::norm(input[late]);
            ++early;
            ++late;
        }
    }
    return result;
}

/*! \brief Reports short-time power variations and near-full-scale input samples. */
void report_cp_input_power(const gr_complex* input, int available, int full_window)
{
    const int count = std::min(available, full_window);
    if (count < 8) {
        return;
    }

    std::array<double, 8> sum{};
    std::array<int, 8> numbers{};
    double max_component = 0.0;
    int near_limit = 0;
    for (int j = 0; j < count; ++j) {
        const int segment = static_cast<int>(8LL * j / count);
        sum[segment] += std::norm(input[j]);
        ++numbers[segment];
        const double maximum = std::max(std::abs(static_cast<double>(input[j].real())),
                                        std::abs(static_cast<double>(input[j].imag())));
        max_component = std::max(max_component, maximum);
        if (maximum >= 0.98) {
            ++near_limit;
        }
    }

    std::fprintf(stderr, "CP_POWER: samples=%d, segments_db=", count);
    for (int i = 0; i < 8; ++i) {
        const double level = 10.0 * std::log10(std::max(sum[i] / numbers[i], 1.0e-20));
        std::fprintf(stderr, "%s%.1f", i == 0 ? "" : ",", level);
    }
    std::fprintf(stderr, ", max_component=%.3f, near_0.98=%d\n",
                 max_component, near_limit);
}

} // namespace

using input_type = gr_complex;
using output_type = gr_complex;

void dvbt_ofdm_synchronization_impl::advance_freq_loop(float error)
{
    d_freq_aux = d_freq_aux + d_beta_freq * error;
    d_fine_freq = d_fine_freq + d_freq_aux + d_alpha_freq * error;
}

void dvbt_ofdm_synchronization_impl::advance_delta_loop(float error)
{
    d_delta_aux = d_delta_aux + d_beta_timing * error;
    d_est_delta = d_est_delta + d_delta_aux + d_alpha_timing * error;
}

int dvbt_ofdm_synchronization_impl::interpolate_input(const gr_complex* in,
                                                      gr_complex* out)
{
    int ii = 0; // Input index
    int oo = 0; // Output index
    double s;
    double f;
    int incr;

    while (oo < d_cp_length + d_fft_length) {
        out[oo++] = d_inter.interpolate(&in[ii], d_samp_phase);
        s = d_samp_phase + d_samp_inc;
        f = floor(s);
        incr = static_cast<int>(f);
        d_samp_phase = s - f;
        ii += incr;
    }

    // Tell the caller how many input values we needed to generate d_cp_length +
    // d_fft_length output values
    return ii;
}

void dvbt_ofdm_synchronization_impl::estimate_fine_synchro(gr_complex* current_channel,
                                                           gr_complex* previous_channel)
{
    gr_complex result_1st = gr_complex(0.0, 0.0);
    volk_32fc_x2_conjugate_dot_prod_32fc(&result_1st,
                                         &current_channel[0],
                                         &previous_channel[0],
                                         floor(d_num_carriers / 2.0));
    gr_complex result_2nd = gr_complex(0.0, 0.0);
    int low = static_cast<int>(floor(d_num_carriers / 2.0));
    volk_32fc_x2_conjugate_dot_prod_32fc(
        &result_2nd, &current_channel[low], &previous_channel[low], d_num_carriers - low);
    float delta_est_error = 1.0 / (1.0 + static_cast<float>(d_cp_length) / d_fft_length) /
                            (2.0 * static_cast<float>(GR_M_PI) * d_num_carriers / 2.0) *
                            std::arg(result_2nd * std::conj(result_1st));
    float freq_est_error = (std::arg(result_1st) + std::arg(result_2nd)) / 2.0 /
                           (1.0 + static_cast<float>(d_cp_length) / d_fft_length);

    // If for any reason the CP position has changed, the signal error is wrong
    // and should not be fed into the loop.
    if (!d_cp_moved) {
        advance_delta_loop(delta_est_error);
        advance_freq_loop(freq_est_error);
    }

    d_samp_inc = 1.0 - d_est_delta;
}

void dvbt_ofdm_synchronization_impl::send_symbol_index_and_resync(int current_offset)
{
    int diff = d_current_symbol - d_previous_symbol;
    // A modulo-four phase discontinuity invalidates downstream symbol alignment.
    if (diff != 1 && diff != -3) {
        // If a symbol skip is detected, we should signal it downstream
        const uint64_t offset = this->nitems_written(0) + current_offset;
        pmt::pmt_t key = pmt::string_to_symbol("resync");
        pmt::pmt_t value =
            pmt::string_to_symbol("generated by sync and channel estimation");
        pmt::pmt_t srcid = pmt::string_to_symbol(blockid);
        this->add_item_tag(0, offset, key, value, srcid);

        d_symbol_correct_count = 0;
    } else {
        ++d_symbol_correct_count;
    }

    // Require four correct transitions before advancing phase without re-estimation.
    d_symbol_acq = d_symbol_correct_count > 3;

    // Signal downstream the relative symbol index found here.
    const uint64_t offset = this->nitems_written(0) + current_offset;
    pmt::pmt_t key = pmt::string_to_symbol("relative_symbol_index");
    pmt::pmt_t value = pmt::from_long(d_current_symbol);
    pmt::pmt_t srcid = pmt::string_to_symbol(blockid);
    this->add_item_tag(0, offset, key, value, srcid);
}

void dvbt_ofdm_synchronization_impl::linearly_estimate_channel_taps(
    int current_symbol, gr_complex* channel_gain)
{
    // This method interpolates scattered measurements across one OFDM symbol
    // It does not use measurements from the previous OFDM symbols (does not use history)
    // as it may have encountered a phase change for the current phase only

    // Current SP carrier
    int current_sp_carrier = 3 * current_symbol;
    // Next SP carrier
    int next_sp_carrier = 12 + 3 * current_symbol;
    for (int i = 0; i < d_num_spilots - 1; ++i) {
#if VOLK_VERSION >= 030100
        volk_32fc_s32fc_multiply2_32fc(&d_aux_linear_estimate_first[0],
                                       &d_coeffs_linear_estimate_first[0],
                                       &channel_gain[current_sp_carrier],
                                       11);
        volk_32fc_s32fc_multiply2_32fc(&d_aux_linear_estimate_last[0],
                                       &d_coeffs_linear_estimate_last[0],
                                       &channel_gain[next_sp_carrier],
                                       11);
#else
        volk_32fc_s32fc_multiply_32fc(&d_aux_linear_estimate_first[0],
                                      &d_coeffs_linear_estimate_first[0],
                                      channel_gain[current_sp_carrier],
                                      11);
        volk_32fc_s32fc_multiply_32fc(&d_aux_linear_estimate_last[0],
                                      &d_coeffs_linear_estimate_last[0],
                                      channel_gain[next_sp_carrier],
                                      11);
#endif
        for (int j = 1; j < 12; ++j) {
            channel_gain[current_sp_carrier + j] =
                d_aux_linear_estimate_first[j - 1] + d_aux_linear_estimate_last[j - 1];
        }
        current_sp_carrier += 12;
        next_sp_carrier += 12;
    }
    // Handle band edges where scattered pilots do not bracket every carrier.

    gr_complex tg_alpha;
    if (current_symbol > 0) {
        // Extrapolate the first pilot interval down to the lowest active carrier.
        current_sp_carrier = 3 * current_symbol;
        next_sp_carrier = 12 + 3 * current_symbol;
        tg_alpha = (channel_gain[next_sp_carrier] - channel_gain[current_sp_carrier]) /
                   gr_complex(next_sp_carrier - current_sp_carrier, 0.0);
        // Calculate interpolation for all intermediate values
        for (int j = -current_sp_carrier; j < 0; ++j) {
            channel_gain[current_sp_carrier + j] =
                channel_gain[current_sp_carrier] + tg_alpha * gr_complex(j, 0.0);
        }
    }

    // Interpolate up to the final continual pilot (CP here means continual pilot).
    current_sp_carrier = 12 * (d_num_spilots - 1) + 3 * current_symbol;
    next_sp_carrier = d_num_carriers - 1;
    float delta_carriers = (float)next_sp_carrier - current_sp_carrier;
    for (int j = 1; j < delta_carriers; ++j) {
        channel_gain[current_sp_carrier + j] =
            channel_gain[current_sp_carrier] * gr_complex(1.0 - j / delta_carriers, 0.0) +
            channel_gain[next_sp_carrier] * gr_complex(j / delta_carriers, 0.0);
    }
}

void dvbt_ofdm_synchronization_impl::calculate_channel_taps_sp(const gr_complex* in,
                                                               int current_symbol,
                                                               gr_complex* channel_gain)
{
    // The modulo-four pilot phase selects which carriers have known values.
    for (int current_sp_carrier = 3 * current_symbol;
         current_sp_carrier < d_num_carriers - 1;
         current_sp_carrier += 12) {
        channel_gain[current_sp_carrier] =
            in[current_sp_carrier] / d_pilot_values[current_sp_carrier];
    }
    // The final active carrier is a continual pilot and anchors the upper edge.
    channel_gain[d_num_carriers - 1] =
        in[d_num_carriers - 1] / d_pilot_values[d_num_carriers - 1];
}

int dvbt_ofdm_synchronization_impl::estimate_symbol_index(const gr_complex* in)
{
    // Find out the OFDM symbol index (value 0 to 3) sent in the
    // current block by correlating scattered pilots with the
    // current block - result is (symbol index % 4)
    float max = 0;
    gr_complex sum = 0;
    int symbol_index = 0;

    int current_sp_carrier; // The current SP carrier
    // sym_count (Symbol count) can take values from 0 to 3, according to the positions of
    // the scattered pilots
    for (int sym_count = 0; sym_count < 4; ++sym_count) {
        sum = 0;
        // The position of the next and current SP carrier based on the value of sym_count
        // For every scattered pilot but the last one...
        for (int next_sp_carrier = 12 + 3 * sym_count;
             next_sp_carrier < d_num_carriers - 1;
             next_sp_carrier += 12) {
            current_sp_carrier = next_sp_carrier - 12;
            if (d_pilot_values[next_sp_carrier] == d_pilot_values[current_sp_carrier]) {
                // Equal PRBS polarities should correlate with the same sign;
                // compensate the known pilot polarity before accumulating.
                sum += in[next_sp_carrier] * conj(in[current_sp_carrier]);
            } else {
                // Opposite PRBS polarities require a sign inversion.
                sum -= in[next_sp_carrier] * conj(in[current_sp_carrier]);
            }

        }
        d_corr_sp[sym_count] = abs(sum);
        if (abs(sum) > max) {
            // When sum is maximum sym_count contains the current symbol (0, 1, 2 or 3)
            max = abs(sum);
            symbol_index = sym_count;
        }
    }
    return symbol_index;
}

void dvbt_ofdm_synchronization_impl::generate_prbs()
{
    // Generate the PRBS sequence for each active carrier.
    // This is actually done by the pilot generator object.
    for (int k = 0; k < d_num_carriers; ++k) {
        d_pilot_values[k] =
            gr_complex((float)(4 * 2 * (0.5 - d_pg.pilot_values()[k])) / 3, 0);
    }
}

int dvbt_ofdm_synchronization_impl::estimate_integer_freq_offset(const gr_complex* in)
{
    // Look for maximum correlation for cpilots in order to obtain post-FFT integer
    // frequency correction
    //
    float max = 0;
    gr_complex sum = 0;
    int start = 0;

    // for d_zeros_on_left +/- d_freq_offset_max...
    for (int i = d_zeros_on_left - d_freq_offset_max;
         i < d_zeros_on_left + d_freq_offset_max;
         ++i) {
        sum = 0;
        for (int j = 0; j < d_num_cpilots - 1; ++j) {
            if (d_known_phase_diff[j] == 0) {
                // Correlate adjacent continual pilots after accounting for
                // their known transmitted polarity relationship.
                sum += in[i + d_cpilots[j + 1]] * conj(in[i + d_cpilots[j]]);
            } else {
                // Reverse the sign for opposite transmitted polarities.
                sum -= in[i + d_cpilots[j + 1]] * conj(in[i + d_cpilots[j]]);
            }
        }
        if (abs(sum) > max) {
            // When sum is maximum, 'i' contains the first active carrier
            max = abs(sum);
            start = i;
        }
    }

    // Stop integer acquisition only after repeated zero residual offsets;
    // a single correlation maximum is insufficient evidence of stable alignment.
    int new_freq_offset = start - d_zeros_on_left;

    if (new_freq_offset == 0) {
        ++d_freq_offset_agree_count;
    } else {
        d_freq_offset_agree_count = 0;
    }

    // Four zero-residual observations establish integer-frequency lock.
    d_freq_offset_acq = d_freq_offset_agree_count > 3;

    return new_freq_offset;
}

void dvbt_ofdm_synchronization_impl::calculate_fft(gr_complex* out)
{
    // Timing and derotation have already populated the FFT input buffer.
    d_fft_calculator.execute();

    // We have to perform an FFT shift
    unsigned int len = (unsigned int)(ceil(d_fft_length / 2.0));
    memcpy(&out[0],
           &d_fft_calculator.get_outbuf()[len],
           sizeof(gr_complex) * (d_fft_length - len));
    memcpy(&out[d_fft_length - len],
           &(d_fft_calculator.get_outbuf()[0]),
           sizeof(gr_complex) * len);
}

bool dvbt_ofdm_synchronization_impl::ml_sync(const gr_complex* in,
                                             int lookup_start,
                                             int lookup_stop,
                                             int* cp_pos,
                                             float* peak_epsilon)
{
    assert(lookup_stop >= d_cp_length + d_fft_length - 1);

    // d_norm and d_corr are sized for one complete acquisition window. A narrow
    // tracking search near its right edge may otherwise exceed these buffers.
    const int max_lookup_start = 2 * d_fft_length + d_cp_length - 1;
    lookup_start = std::min(lookup_start, max_lookup_start);
    d_diag_cp_rejection = {};
    if (lookup_start <= lookup_stop) {
        d_cp_quality = 0.0f;
        d_diag_cp_rejection.reason = "empty-window";
        return false;
    }

    const int search_size = lookup_start - lookup_stop;
    const int first_sample = lookup_stop - d_cp_length + 1;
    const int first_corr = first_sample - d_fft_length;

    // Compute sample energies once. Each CP candidate compares two windows
    // separated by d_fft_length samples, so both windows use this same array.
    const int norm_count = lookup_start - first_corr + 1;
    volk_32fc_magnitude_squared_32f(&d_norm[first_corr], &in[first_corr], norm_count);

    // Compute the complex products x[n] * conj(x[n - FFT]) once with VOLK.
    // d_corr is indexed by n - FFT to keep all indices non-negative.
    const int product_count = lookup_start - first_sample + 1;
    volk_32fc_x2_multiply_conjugate_32fc(&d_corr[first_corr],
                                         &in[first_sample],
                                         &in[first_corr],
                                         product_count);

    // Initialize the first CP window with VOLK accumulators. The correlation
    // and both energy sums each cover exactly d_cp_length input samples.
    gr_complex gamma{};
    float energy_late = 0.0f;
    float energy_early = 0.0f;
    volk_32fc_accumulator_s32fc(&gamma, &d_corr[first_corr], d_cp_length);
    volk_32f_accumulator_s32f(&energy_late, &d_norm[first_sample], d_cp_length);
    volk_32f_accumulator_s32f(&energy_early, &d_norm[first_corr], d_cp_length);

    d_gamma[0] = gamma;
    d_phi[0] = energy_late + energy_early;

    // Neighboring CP windows overlap in all but one sample. Instead of
    // summing the whole CP again at every position, remove the oldest term
    // and add the newest one. This reduces the search from O(FFT * CP)
    // operations to O(FFT + CP), especially important during reacquisition.
    for (int k = 1; k < search_size; ++k) {
        const int outgoing = first_sample + k - 1;
        const int incoming = outgoing + d_cp_length;

        gamma += d_corr[incoming - d_fft_length] - d_corr[outgoing - d_fft_length];
        energy_late += d_norm[incoming] - d_norm[outgoing];
        energy_early += d_norm[incoming - d_fft_length] -
                        d_norm[outgoing - d_fft_length];

        d_gamma[k] = gamma;
        d_phi[k] = energy_late + energy_early;
    }

    // The energy penalty reduces false peaks caused by power variations:
    // lambda = abs(gamma) - rho * (E_early + E_late) / 2.
    volk_32fc_magnitude_32f(&d_lambda[0], &d_gamma[0], search_size);
    volk_32f_s32f_multiply_32f(&d_phi[0], &d_phi[0], d_rho / 2.0f, search_size);
    volk_32f_x2_subtract_32f(&d_lambda[0], &d_lambda[0], &d_phi[0], search_size);

    // Find the maximum of the ML metric.
    uint16_t candidate = 0;
    volk_32f_index_max_16u(&candidate, &d_lambda[0], search_size);

    // Calculate normalized CP correlation.
    // d_phi already contains (E1 + E2) * d_rho / 2.
    const float correlation = std::abs(d_gamma[candidate]);
    const float weighted_energy = d_phi[candidate];

    d_cp_quality = 0.0f;
    d_diag_cp_rejection.peak_index = candidate;
    d_diag_cp_rejection.search_size = search_size;
    d_diag_cp_rejection.peak_metric = d_lambda[candidate];

    if (d_rho > 0.0f && weighted_energy > 0.0f &&
        std::isfinite(correlation) && std::isfinite(weighted_energy)) {
        d_cp_quality = d_rho * correlation / weighted_energy;
    }

    // Initial acquisition is confirmed over three consecutive OFDM symbols
    // in general_work(), rather than requiring a high single-peak threshold.
    constexpr float min_cp_quality = 0.30f;

    d_diag_cp_rejection.quality = d_cp_quality;
    if (!std::isfinite(d_cp_quality) || d_cp_quality < min_cp_quality) {
        d_diag_cp_rejection.reason = "low-quality";
        return false;
    }

    // In tracking mode, a sudden amplitude change can invalidate the absolute
    // ML peak threshold even though the CP correlation remains strong and its
    // position is still correct. Do not discard a well-established lock for
    // that reason alone. Initial acquisition remains unchanged.
    int peak = 0;
    if (!detect_peak_position(&d_lambda[0], search_size, &peak)) {
        const bool reliable_tracking_peak =
            d_initial_acquired && d_cp_quality >= 0.85f &&
            d_diag_cp_rejection.reason == std::string("adaptive-level") &&
            std::abs(peak + lookup_stop - d_cp_start) <= 8;
        if (!reliable_tracking_peak) {
            return false;
        }
        d_avg_max = d_lambda[peak];
        ++d_diag_peak_overrides;
        if (d_diag_peak_overrides == 1 || d_diag_peak_overrides % 100 == 0) {
            std::fprintf(stderr,
                         "CP_PEAK_OVERRIDE: quality=%.3f, distance=%d, total=%" PRIu64 "\n",
                         d_cp_quality, peak + lookup_stop - d_cp_start,
                         d_diag_peak_overrides);
        }
    }

    *cp_pos = peak + lookup_stop;
    *peak_epsilon = fast_atan2f(d_gamma[peak]);

    return true;
}

void dvbt_ofdm_synchronization_impl::init_peak_detection(float threshold_factor_rise,
                                                         float alpha)
{
    d_avg_alpha = alpha;
    d_threshold_factor_rise = threshold_factor_rise;

    d_avg_max = -std::numeric_limits<float>::infinity();
    d_avg_min = std::numeric_limits<float>::infinity();
}

bool dvbt_ofdm_synchronization_impl::detect_peak_position(const float* data,
                                                          const int data_size,
                                                          int* peak_pos)
{
    uint16_t peak_index = 0;
    uint32_t d_data_size = static_cast<uint32_t>(data_size);
    bool success = true;
    static unsigned diag_checks = 0;
    static unsigned diag_level_rejects = 0;
    static unsigned diag_border_rejects = 0;

    // Find the maximum value in the given data vector
    volk_32f_index_max_16u(&peak_index, &data[0], d_data_size);

    // Reject peaks at the border of the search interval.
    const bool at_border = data_size < d_fft_length
                               ? peak_index == 0 || peak_index == data_size - 1
                               : peak_index < 5 || peak_index > data_size - 5;

    if (at_border) {
        success = false;
        ++diag_border_rejects;
        d_diag_cp_rejection.reason = "border";
    } else {
        if (data_size >= d_fft_length) {
            float min = data[(peak_index + d_fft_length / 2) % d_fft_length];

            if (d_avg_min == static_cast<float>(INFINITY)) {
                d_avg_min = min;
            } else {
                d_avg_min = d_avg_alpha * min + (1 - d_avg_alpha) * d_avg_min;
            }
        }

        if (d_avg_max == static_cast<float>(-INFINITY)) {
            d_avg_max = data[peak_index];
        } else if (data[peak_index] >
                   d_avg_max - d_threshold_factor_rise * (d_avg_max - d_avg_min)) {
            d_avg_max = d_avg_alpha * data[peak_index] + (1 - d_avg_alpha) * d_avg_max;
        } else {
            success = false;
            ++diag_level_rejects;
            d_diag_cp_rejection.reason = "adaptive-level";
        }
    }

    if (d_avg_max != -std::numeric_limits<float>::infinity() &&
        d_avg_min != std::numeric_limits<float>::infinity()) {
        d_diag_cp_rejection.threshold =
            d_avg_max - d_threshold_factor_rise * (d_avg_max - d_avg_min);
    }
    *peak_pos = peak_index;

    if (++diag_checks >= 200) {
#if 0
        std::fprintf(stderr, "Peak detection: level=%u, border=%u, checks=%u\n",
                    diag_level_rejects,
                    diag_border_rejects,
                    diag_checks);
        std::fflush(stderr);
#endif

        diag_checks = 0;
        diag_level_rejects = 0;
        diag_border_rejects = 0;
    }

    return success;
}

void dvbt_ofdm_synchronization_impl::send_sync_start(int current_offset)
{
    const uint64_t offset = this->nitems_written(0) + current_offset;
    pmt::pmt_t key = pmt::string_to_symbol("sync_start");
    pmt::pmt_t value = pmt::from_long(1);
    pmt::pmt_t srcid = pmt::string_to_symbol("dvbt_ofdm_synchronization");
    this->add_item_tag(0, offset, key, value, srcid);
}

// Derotates the signal
void dvbt_ofdm_synchronization_impl::derotate(const gr_complex* in, gr_complex* out)
{
    double sensitivity = (double)(-1) / (double)d_fft_length;
    d_phaseinc = sensitivity * d_total_freq_error;

    gr_complex phase_increment = gr_complex(std::cos(d_phaseinc), std::sin(d_phaseinc));
    gr_complex phase_current = gr_complex(std::cos(d_phase), std::sin(d_phase));

#if VOLK_VERSION >= 030100
    volk_32fc_s32fc_x2_rotator2_32fc(
        &out[0], &in[0], &phase_increment, &phase_current, d_fft_length);
#else
    volk_32fc_s32fc_x2_rotator_32fc(
        &out[0], &in[0], phase_increment, &phase_current, d_fft_length);
#endif
    d_phase = std::arg(phase_current);
    d_phase = fmod(d_phase + d_phaseinc * d_cp_length, (float)2 * M_PI);
}

// Resets the state variables after synchronization loss.
void dvbt_ofdm_synchronization_impl::reset_synchronization()
{
    d_initial_acquired = false;
    d_cp_found = false;
    d_cp_moved = true;
    d_acquisition_matches = 0;
    d_acquisition_cp_position = 0;

    d_symbol_acq = false;
    d_current_symbol = 0;
    d_previous_symbol = 0;
    d_symbol_correct_count = 0;
    d_diag_sp_mismatch_run = 0;

    d_freq_offset = 0;
    d_freq_offset_acq = false;
    d_freq_offset_agree_count = 0;

    d_coarse_freq = 0.0f;
    d_fine_freq = 0.0f;
    d_freq_aux = 0.0f;
    d_total_freq_error = 0.0f;

    d_est_delta = 0.0f;
    d_delta_aux = 0.0f;
    d_samp_inc = 1.0f;
    d_samp_phase = 0.0f;

    d_phase = 0.0f;
    d_phaseinc = 0.0;

    init_peak_detection(0.3f, 0.9f);
}

dvbt_ofdm_synchronization::sptr
dvbt_ofdm_synchronization::make(dvbt_transmission_mode_t transmission_mode,
                                dvb_guardinterval_t guard_interval,
                                float snr,
                                bool interpolate,
                                double sample_rate,
                                float squelch_db,
                                bool diagnostics)
{
    return gnuradio::make_block_sptr<dvbt_ofdm_synchronization_impl>(
        transmission_mode, guard_interval, snr, interpolate, sample_rate,
        squelch_db, diagnostics);
}

dvbt_ofdm_synchronization_impl::dvbt_ofdm_synchronization_impl(
    dvbt_transmission_mode_t transmission_mode,
    dvb_guardinterval_t guard_interval,
    float snr,
    bool interpolate,
    double sample_rate,
    float squelch_db,
    bool diagnostics)
    : gr::block("dvbt_ofdm_synchronization",
                gr::io_signature::make(1, 1, sizeof(input_type)),
                gr::io_signature::make3(
                    1,
                    4,
                    sizeof(output_type) * get_num_carriers(transmission_mode),
                    sizeof(output_type) * get_num_carriers(transmission_mode),
                    sizeof(float))),
      d_config(dtv::MOD_16QAM,
               dtv::NH,
               dtv::C1_2,
               dtv::C1_2,
               guard_interval,
               transmission_mode),
      d_pg(d_config),

      d_num_carriers{ d_config.d_Kmax - d_config.d_Kmin + 1 },
      d_num_cpilots{ d_pg.cpilot_carriers_size() },
      d_num_spilots{ d_pg.spilot_carriers_size() },
      d_cpilots{ d_pg.cpilot_carriers() },

      d_fft_length{ d_config.d_fft_length },
      d_cp_length{ d_config.d_cp_length },
      d_sample_rate{ sample_rate },
      d_squelch_db{ squelch_db },
      d_diagnostics{ diagnostics },

      d_cp_start{ 0 },
      d_cp_found{ 0 },
      d_gamma(d_fft_length + d_cp_length),
      d_phi(d_fft_length),
      d_lambda(d_fft_length),
      d_coarse_freq{ 0.0 },
      d_derot(d_fft_length + d_cp_length),
      d_conj(2 * d_fft_length + d_cp_length),
      d_norm(2 * d_fft_length + d_cp_length),
      d_corr(2 * d_fft_length + d_cp_length),

      d_snr{ snr },
      d_rho{ calculate_rho(d_snr) },

      d_threshold_factor_rise{ 0.0 },
      d_avg_alpha{ 0.0 },
      d_avg_min{ 0.0 },
      d_avg_max{ 0.0 },

      d_phaseinc{ 0.0 },
      d_nextphaseinc{ 0.0 },
      d_nextpos{ 0 },
      d_phase{ 0.0 },

      d_initial_acquired{ false },

      d_postfft(d_fft_length),
      d_fft_calculator(d_fft_length),

      // Integer frequency correction part
      d_zeros_on_left{ d_config.d_zeros_on_left },
      d_freq_offset_max{ 10 },
      d_freq_offset{ 0 },
      d_freq_offset_agree_count{ 0 },
      d_freq_offset_acq{ false },

      d_known_phase_diff(d_num_cpilots),
      d_integer_freq_derotated{ nullptr },

      d_pilot_values(d_num_carriers),
      d_channel_gain(d_num_carriers),
      d_coeffs_linear_estimate_first(11), // Eleven interior bins between pilots spaced by twelve.
      d_aux_linear_estimate_first(11),    // Eleven interior bins between pilots spaced by twelve.
      d_coeffs_linear_estimate_last(11),  // Eleven interior bins between pilots spaced by twelve.
      d_aux_linear_estimate_last(11),     // Eleven interior bins between pilots spaced by twelve.
      d_channel_gain_mag_sq(d_num_carriers),
      d_ones(d_num_carriers),
      d_channel_gain_inv(d_num_carriers),

      d_current_symbol{ 0 },
      d_previous_symbol{ 0 },
      d_symbol_acq{ false },
      d_symbol_correct_count{ 0 },
      d_corr_sp(4), // One correlation score per scattered-pilot phase.

      d_previous_channel_gain(d_num_carriers),
      d_delta_channel_gains(d_num_carriers),
      d_interpolate{ interpolate },
      d_interpolated(d_fft_length + d_cp_length),
      d_inter{ gr::filter::mmse_fir_interpolator_cc() },
      d_samp_inc{ 1.0 },
      d_samp_phase{ 0.0 },
      d_cp_start_offset{ -10 },

      d_alpha_freq{ 0.0 },
      d_beta_freq{ 0.0 },
      d_alpha_timing{ 0.0 },
      d_beta_timing{ 0.0 },
      d_freq_aux{ 0.0 },
      d_fine_freq{ 0.0 },
      d_delta_aux{ 0.0 },
      d_est_delta{ 0.0 },
      d_cp_moved{ true },

      d_total_freq_error{ 0.0 }
{
    if (!std::isfinite(sample_rate) || sample_rate <= 0.0) {
        throw std::invalid_argument("DVB-T sample rate must be positive and finite");
    }
    if (!std::isfinite(squelch_db)) {
        throw std::invalid_argument("DVB-T squelch threshold must be finite");
    }

    set_relative_rate(1.0 / static_cast<double>(d_cp_length + d_fft_length));

    init_peak_detection(0.3, 0.9);

    generate_prbs();

    // Cache expected polarity differences between adjacent continual pilots.
    for (int i = 0; i < (d_num_cpilots - 1); ++i) {
        d_known_phase_diff[i] =
            std::norm(d_pilot_values[d_cpilots[i + 1]] - d_pilot_values[d_cpilots[i]]);
    }

    for (int i = 0; i < 4; ++i) {
        d_corr_sp[i] = 0.0;
    }

    for (int i = 0; i < d_num_carriers; ++i) {
        d_ones[i] = 1.0;
    }

    for (int i = 1; i < 12; ++i) {
        d_coeffs_linear_estimate_first[i - 1] = gr_complex(1.0 - i / 12.0, 0.0);
        d_coeffs_linear_estimate_last[i - 1] = gr_complex(i / 12.0, 0.0);
    }

    float loop_bw_freq = GR_M_PI / 100;
    float loop_bw_timing = GR_M_PI / 10000;
    float damping = sqrtf(2.0f) / 2.0f;
    float denom = (1.0 + 2.0 * damping * loop_bw_freq + loop_bw_freq * loop_bw_freq);
    d_beta_freq = (4 * loop_bw_freq * loop_bw_freq) / denom;
    d_alpha_freq = (4 * damping * loop_bw_freq) / denom;
    denom = (1.0 + 2.0 * damping * loop_bw_timing + loop_bw_timing * loop_bw_timing);
    d_beta_timing = (4 * loop_bw_timing * loop_bw_timing) / denom;
    d_alpha_timing = (4 * damping * loop_bw_timing) / denom;
    d_freq_aux = 0;
    d_fine_freq = 0;
    d_total_freq_error = 0;
}

dvbt_ofdm_synchronization_impl::~dvbt_ofdm_synchronization_impl() {}

/*!
 * @brief Changes the nominal input sample rate used for the frequency-error display.
 * @param sample_rate New input sample rate after resampling, in samples per second.
 * @throws std::invalid_argument If sample_rate is not positive or finite.
 *
 * This setting changes only the conversion of frequency error to Hz. The current
 * synchronization and tracking state is not reset.
 */
void dvbt_ofdm_synchronization_impl::set_sample_rate(double sample_rate)
{
    if (!std::isfinite(sample_rate) || sample_rate <= 0.0) {
        throw std::invalid_argument("DVB-T sample rate must be positive and finite");
    }
    d_sample_rate.store(sample_rate, std::memory_order_relaxed);
}

/*!
 * @brief Returns the configured nominal sample rate in samples per second.
 * @return Input sample rate after resampling, in samples per second.
 */
double dvbt_ofdm_synchronization_impl::get_sample_rate() const
{
    return d_sample_rate.load(std::memory_order_relaxed);
}

/*!
 * @brief Changes the squelch threshold while the receiver is running.
 *
 * @param threshold_db Digital IQ threshold in dBFS, or -200 to disable.
 */
void dvbt_ofdm_synchronization_impl::set_squelch_threshold(float threshold_db)
{
    if (!std::isfinite(threshold_db)) {
        throw std::invalid_argument("DVB-T squelch threshold must be finite");
    }
    d_squelch_db.store(threshold_db, std::memory_order_relaxed);
}

/*!
 * @brief Returns the current squelch threshold in dBFS.
 */
float dvbt_ofdm_synchronization_impl::get_squelch_threshold() const
{
    return d_squelch_db.load(std::memory_order_relaxed);
}

void dvbt_ofdm_synchronization_impl::forecast(int noutput_items,
                                              gr_vector_int& ninput_items_required)
{
    // Include acquisition look-ahead and the interpolator tap window.
    int ninputs = ninput_items_required.size();
    for (int i = 0; i < ninputs; ++i) {
        ninput_items_required[i] = static_cast<int>(
            ceil((d_cp_length + d_fft_length) * (noutput_items + 1) * d_samp_inc) +
            d_inter.ntaps());
    }
}

int dvbt_ofdm_synchronization_impl::general_work(int noutput_items,
                                                 gr_vector_int& ninput_items,
                                                 gr_vector_const_void_star& input_items,
                                                 gr_vector_void_star& output_items)
{
    auto in = static_cast<const input_type*>(input_items[0]);
    auto out = static_cast<output_type*>(output_items[0]);

    bool ch_output_connected = output_items.size() >= 2;
    bool freq_error_output_connected = output_items.size() >= 3;
    bool samp_error_output_connected = output_items.size() >= 4;

    output_type* out_channel_gain;
    float* out_freq_error;
    float* out_samp_error;
    if (ch_output_connected) {
        out_channel_gain = static_cast<output_type*>(output_items[1]);
    }
    if (freq_error_output_connected) {
        out_freq_error = static_cast<float*>(output_items[2]);
    }
    if (samp_error_output_connected) {
        out_samp_error = static_cast<float*>(output_items[3]);
    }

    d_consumed = 0;
    d_to_output = 0;
    const uint64_t input_base = nitems_read(0);

    for (int i = 0; i < noutput_items; ++i) {
        int required_for_interpolation = d_fft_length + d_cp_length;
        int observed_cp_position = -1;
        float narrow_cp_quality = 0.0f;
        cp_rejection_diagnostic narrow_cp_rejection;

        // Measure input power before CP processing; also used by the squelch.
        double power = 0.0;
        for (int j = 0; j < 256; ++j) {
            power += std::norm(in[d_consumed + j]);
        }
        const double mean_power = power / 256.0;
        const double power_db = 10.0 * std::log10(std::max(mean_power, 1.0e-20));
        const uint64_t power_sample = input_base + d_consumed + 128;
        const float squelch_db = d_squelch_db.load(std::memory_order_relaxed);
        const bool squelch_enabled = squelch_db > -200.0f;

        // The digital-IQ level is not a calibrated RF power in dBm.
        // Hysteresis prevents repeated transitions around the threshold.
        if (!d_input_power_initialized) {
            d_input_power_initialized = true;
            d_input_signal_present = !squelch_enabled || power_db >= squelch_db;
            std::fprintf(stderr,
                         "SQUELCH_INITIAL: sample=%" PRIu64
                         ", power=%.1f dBFS, open=%d, threshold=%.1f dBFS\n",
                         power_sample, power_db, d_input_signal_present, squelch_db);
        } else if (squelch_enabled && d_input_signal_present &&
                   power_db <= squelch_db - 3.0) {
            d_input_signal_present = false;
            d_signal_return_seen = false;
            if (d_initial_acquired) {
                send_sync_start(d_to_output);
            }
            reset_synchronization();
            std::fprintf(stderr,
                         "SQUELCH_CLOSED: sample=%" PRIu64 ", power=%.1f dBFS\n",
                         power_sample, power_db);
        } else if (!d_input_signal_present &&
                   (!squelch_enabled || power_db >= squelch_db)) {
            d_input_signal_present = true;
            d_signal_return_seen = true;
            d_signal_return_sample = power_sample;
            d_diag_full_searches = 0;
            d_diag_first_candidates = 0;
            d_diag_confirmation_failures = 0;
            std::fprintf(stderr,
                         "SQUELCH_OPENED: sample=%" PRIu64 ", power=%.1f dBFS\n",
                         power_sample, power_db);
        }

        if (!d_input_signal_present) {
            // Skip large contiguous chunks of noise without searching for CPs.
            // Do not emit data. On closing, inform downstream once and reset.
            const int remaining = ninput_items[0] - d_consumed;
            d_consumed += std::min(remaining, 8192);
            consume_each(d_consumed);
            return d_to_output;
        }

        if (!d_initial_acquired) {
            // A single valid CP correlation is not sufficient for acquisition.
            // Confirm a candidate at the same OFDM symbol position twice more
            // before updating the receiver loops or producing output symbols.
            constexpr int required_matches = 3;
            constexpr int confirmation_half_width = 32;
            const int symbol_length = d_fft_length + d_cp_length;
            const int min_cp_position = symbol_length - 1;
            const int centered_cp_position = min_cp_position + d_fft_length / 2;

            int candidate_cp_position = 0;
            float candidate_coarse_freq = 0.0f;
            bool candidate_found = false;

            if (d_acquisition_matches == 0) {
                ++d_diag_full_searches;
                // Find the first CP over the complete acquisition window.
                candidate_found = ml_sync(&in[d_consumed],
                                          2 * d_fft_length + d_cp_length - 1,
                                          min_cp_position,
                                          &candidate_cp_position,
                                          &candidate_coarse_freq);
            } else {
                // A real CP must recur one complete OFDM symbol later.
                // Re-check near the expected position, allowing for timing error.
                candidate_found = ml_sync(&in[d_consumed],
                                          d_acquisition_cp_position + confirmation_half_width,
                                          d_acquisition_cp_position - confirmation_half_width,
                                          &candidate_cp_position,
                                          &candidate_coarse_freq);
            }

            if (candidate_found) {
                if (d_acquisition_matches == 0) {
                    ++d_diag_first_candidates;
                }
                ++d_acquisition_matches;

                if (d_acquisition_matches < required_matches) {
                    // Shift the next expected peak to the center of the available
                    // search window. Otherwise a peak near the edge of the full
                    // search interval could fail the narrow border check.
                    // The shift is based on the actual candidate position, so
                    // the next OFDM symbol is expected at centered_cp_position.
                    d_acquisition_cp_position = centered_cp_position;
                    d_consumed += symbol_length + candidate_cp_position -
                                  centered_cp_position;
                    consume_each(d_consumed);
                    return d_to_output;
                }

                // Three consistent CP positions were found. Only the final
                // symbol is decoded; no tentative symbol reaches downstream.
                // Use the last input sample needed by the third CP search.
                const uint64_t acquired_sample =
                    input_base + d_consumed + centered_cp_position + confirmation_half_width;
                const uint64_t cp_sample = input_base + d_consumed + candidate_cp_position;
                std::fprintf(stderr,
                             "CP_ACQUIRED: sample=%" PRIu64 ", cp_sample=%" PRIu64
                             ", power=%.1f dB, full_searches=%" PRIu64
                             ", first_candidates=%" PRIu64
                             ", confirmation_failures=%" PRIu64,
                             acquired_sample, cp_sample, power_db,
                             d_diag_full_searches, d_diag_first_candidates,
                             d_diag_confirmation_failures);
                if (d_signal_return_seen && acquired_sample >= d_signal_return_sample) {
                    std::fprintf(stderr, ", since_return=%" PRIu64 " samples",
                                 acquired_sample - d_signal_return_sample);
                }
                if (d_diag_await_reacquisition && d_diag_last_good_valid &&
                    cp_sample >= d_diag_last_good_cp_sample) {
                    const uint64_t since_good = cp_sample - d_diag_last_good_cp_sample;
                    const int symbol_length = d_fft_length + d_cp_length;
                    const int remainder = static_cast<int>(since_good % symbol_length);
                    const int phase_residual = remainder > symbol_length / 2
                                                   ? remainder - symbol_length : remainder;
                    std::fprintf(stderr, ", since_good=%" PRIu64
                                         ", phase_residual=%d",
                                 since_good, phase_residual);
                }
                std::fprintf(stderr, "\n");
                std::fflush(stderr);
                d_diag_await_reacquisition = false;
                observed_cp_position = candidate_cp_position;
                d_initial_acquired = true;
                d_cp_found = true;
                d_cp_start = candidate_cp_position;
                d_coarse_freq = candidate_coarse_freq;
                d_acquisition_matches = 0;
            } else {
                if (d_acquisition_matches > 0) {
                    ++d_diag_confirmation_failures;
                }
                // Discard the tentative acquisition and resume the full search.
                d_acquisition_matches = 0;
                d_cp_found = false;
            }

            // Restart interpolation and symbol/frequency acquisition when a
            // newly confirmed signal enters the normal processing path.
            d_samp_phase = 0.0f;
            d_cp_moved = true;
            d_symbol_acq = false;
            d_freq_offset_acq = false;
        } else {
            // Verify the tracked position with a narrow CP search. Its timing and
            // frequency estimates are not applied: fine loops provide continuity.
            // Only the wider fallback below may replace the stored CP position.
            int cp_start_temp;
            float coarse_freq_temp;
            d_cp_found = ml_sync(&in[d_consumed],
                                 d_cp_start + 8,
                                 std::max(d_cp_start - 8, d_cp_length + d_fft_length - 1),
                                 &cp_start_temp,
                                 &coarse_freq_temp);

            narrow_cp_quality = d_cp_quality;
            narrow_cp_rejection = d_diag_cp_rejection;
            if (d_cp_found) {
                observed_cp_position = cp_start_temp;
            }
            if (!d_cp_found) {
                // Retry with a larger search range
                d_cp_found =
                    ml_sync(&in[d_consumed],
                            d_cp_start + 16,
                            std::max(d_cp_start - 16, d_cp_length + d_fft_length - 1),
                            &d_cp_start,
                            &coarse_freq_temp);

								if (d_cp_found) {
										// A moved FFT window requires fresh fractional timing and loop gating.
                		d_samp_phase = 0.0f;
                		d_cp_moved = true;
                        observed_cp_position = d_cp_start;
								}
            }
        }

#if 0
        // Disabled aggregate CP diagnostic; retained for targeted measurements.
        // Diagnostic: CP detection and normalized correlation.
        static unsigned cp_checks = 0;
        static unsigned cp_hits = 0;
        static double cp_quality_sum = 0.0;
        static float cp_quality_min = 1.0f;
        static float cp_quality_max = 0.0f;
        static unsigned cp_quality_hits = 0;
        static double input_power_sum = 0.0;

        ++cp_checks;
        input_power_sum += mean_power;

        if (d_cp_found) {
            ++cp_hits;
        }
        if (d_cp_quality >= 0.30f) {
            ++cp_quality_hits;
        }

        cp_quality_sum += d_cp_quality;
        cp_quality_min = std::min(cp_quality_min, d_cp_quality);
        cp_quality_max = std::max(cp_quality_max, d_cp_quality);

        if (cp_checks >= 200) {
            std::printf("CP: %u/%u found, quality OK: %u/%u, acquired=%d, "
                        "cp_start=%d, coarse_freq=%f, freq_offset=%d, "
                        "quality=%.3f / %.3f / %.3f (min/avg/max)\n",
                        cp_hits, cp_checks,
                        cp_quality_hits, cp_checks,
                        d_initial_acquired,
                        d_cp_start,
                        d_coarse_freq,
                        d_freq_offset,
                        cp_quality_min,
                        cp_quality_sum / cp_checks,
                        cp_quality_max);

                        const double mean_power = input_power_sum / cp_checks;
                        std::printf("Input power: %.1f dB\n",
                                    10.0 * std::log10(std::max(mean_power, 1.0e-20)));
                        input_power_sum = 0.0;

                        cp_checks = 0;
                        cp_hits = 0;
                        cp_quality_sum = 0.0;
                        cp_quality_min = 1.0f;
                        cp_quality_max = 0.0f;
                        cp_quality_hits = 0;
        }
#endif

        if (d_cp_found) {
            // Retain only diagnostic history; the synchronization state is unchanged.
            if (observed_cp_position >= 0) {
                d_diag_last_good_cp_sample = input_base + d_consumed + observed_cp_position;
                d_diag_last_good_valid = true;
            }
            d_diag_last_good_quality = d_cp_quality;
            d_diag_last_good_power_db = power_db;
            d_diag_recent_qualities[d_diag_recent_next] = d_cp_quality;
            d_diag_recent_next =
                (d_diag_recent_next + 1) % d_diag_recent_qualities.size();
            d_diag_recent_count =
                std::min(d_diag_recent_count + 1, d_diag_recent_qualities.size());

            // Move the FFT window ten samples earlier to reduce the risk of
            // including samples from the next symbol. This fixed margin relies
            // on the cyclic extension and is not a measured channel delay.
            d_cp_start_offset = -10;

            int low = d_cp_start + d_cp_start_offset - d_fft_length + 1;

            if (d_interpolate) {
                // Compensate the MMSE interpolator's reference-tap alignment:
                // its tap window starts three samples before the desired sample.
                // Keep this offset consistent with the interpolator implementation.
                required_for_interpolation =
                    interpolate_input(&in[d_consumed + low - 3], &d_interpolated[0]);

                // Derotate the resampled useful-symbol window.
                derotate(&d_interpolated[0], d_fft_calculator.get_inbuf());
            } else {
                derotate(&in[d_consumed + low], d_fft_calculator.get_inbuf());
            }

            // Transform the corrected useful-symbol window into centered FFT bins.
            calculate_fft(&d_postfft[0]);

            // Calculate the integer frequency error (only when not already acquired)
            int current_freq_offset = 0;
            if (!d_freq_offset_acq) {
                current_freq_offset = estimate_integer_freq_offset(&d_postfft[0]);
            }
            // Accumulate the residual shift because future derotation removes it.
            d_freq_offset += current_freq_offset;

            // correct the integer frequency offset (point to the shifted position)
            d_integer_freq_derotated =
                &d_postfft[0] + current_freq_offset + d_zeros_on_left;

            // Estimate the current symbol index.
            // Only re-estimate it when the symbol was not acquired.
            d_previous_symbol = d_current_symbol;
            if (d_symbol_acq) {
                d_current_symbol = (d_current_symbol + 1) % 4;
            } else {
                d_current_symbol = estimate_symbol_index(d_integer_freq_derotated);
            }
            send_symbol_index_and_resync(i);

            // Independently measure the pilot phase on every acquired OFDM symbol.
            // This check does not modify d_current_symbol, equalization or stream tags.
            if (d_symbol_acq) {
                const int observed_phase = estimate_symbol_index(d_integer_freq_derotated);
                ++d_diag_sp_checks;
                const float expected_score = d_corr_sp[d_current_symbol];
                const float observed_score = d_corr_sp[observed_phase];
                constexpr float min_sp_score_ratio = 1.25f;
                const bool strong_mismatch = observed_phase != d_current_symbol &&
                    observed_score > min_sp_score_ratio * expected_score;
                if (strong_mismatch) {
                    ++d_diag_sp_mismatches;
                    ++d_diag_sp_mismatch_run;
                    if (d_diag_sp_mismatch_run == 1 || d_diag_sp_mismatch_run == 4 ||
                        d_diag_sp_mismatch_run % 64 == 0) {
                        std::fprintf(stderr,
                                     "SP_MISMATCH: out_symbol=%" PRIu64
                                     ", expected=%d, observed=%d, score_expected=%.3g"
                                     ", score_observed=%.3g, run=%" PRIu64
                                     ", total=%" PRIu64 "\n",
                                     nitems_written(0) + i, d_current_symbol,
                                     observed_phase, expected_score, observed_score,
                                     d_diag_sp_mismatch_run, d_diag_sp_mismatches);
                    }
                } else if (d_diag_sp_mismatch_run > 0) {
                    std::fprintf(stderr,
                                 "SP_RECOVERED: out_symbol=%" PRIu64
                                 ", mismatch_run=%" PRIu64 ", checks=%" PRIu64 "\n",
                                 nitems_written(0) + i, d_diag_sp_mismatch_run,
                                 d_diag_sp_checks);
                    d_diag_sp_mismatch_run = 0;
                }
            }

            // Swap buffer ownership: retain the preceding estimate for fine
            // tracking and reuse its old storage for the new estimate. Vector
            // swap exchanges allocations; it does not copy all carrier values.
            d_channel_gain.swap(d_previous_channel_gain);

            // Measure channel gain at the scattered-pilot positions...
            calculate_channel_taps_sp(
                d_integer_freq_derotated, d_current_symbol, &d_channel_gain[0]);

            // and interpolate in the rest of the carriers.
            linearly_estimate_channel_taps(d_current_symbol, &d_channel_gain[0]);

            // Zero-forcing equalization: Y/H = Y*conj(H)/|H|^2. Split the
            // operation into VOLK kernels supported by the targeted versions.
            volk_32fc_x2_multiply_conjugate_32fc(&out[i * d_num_carriers],
                                                 d_integer_freq_derotated,
                                                 &d_channel_gain[0],
                                                 d_num_carriers);
            volk_32fc_magnitude_squared_32f(
                &d_channel_gain_mag_sq[0], &d_channel_gain[0], d_num_carriers);
            volk_32f_x2_divide_32f(&d_channel_gain_mag_sq[0],
                                   &d_ones[0],
                                   &d_channel_gain_mag_sq[0],
                                   d_num_carriers);
            volk_32fc_32f_multiply_32fc(&out[i * d_num_carriers],
                                        &out[i * d_num_carriers],
                                        &d_channel_gain_mag_sq[0],
                                        d_num_carriers);
            if (ch_output_connected) {
                // The channel taps output is connected
                memcpy(&out_channel_gain[i * d_num_carriers],
                       &d_channel_gain[0],
                       d_num_carriers * sizeof(gr_complex));
            }
            if (freq_error_output_connected) {
                // Display receiver frequency minus signal frequency, in Hz.
                out_freq_error[i] = static_cast<float>(
                    -(d_total_freq_error / (2.0 * GR_M_PI)) *
                    (get_sample_rate() / d_fft_length));
            }
            if (samp_error_output_connected) {
                // Convert relative sampling-rate error to ppm.
                out_samp_error[i] = d_est_delta * 1.0e6f;
            }

            // This is the value used for derotation. Attention should be payed, since
            // it includes coarse frequency, fine frequency AND integer frequency offset.
            d_total_freq_error = d_fine_freq + d_coarse_freq +
                                 static_cast<float>(GR_M_PI) * 2 * d_freq_offset;

            // Feed successive channel estimates into the residual tracking loops.
            estimate_fine_synchro(&d_channel_gain[0], &d_previous_channel_gain[0]);

            d_to_output += 1;

            // After a large frequency step, pilot-based fine tracking may lose its
            // unambiguous range while CP correlation remains good. Do not let the
            // integrator continue to grow without re-acquiring integer frequency.
            constexpr float max_fine_freq = 2.0f * static_cast<float>(GR_M_PI);
            if (!std::isfinite(d_fine_freq) || !std::isfinite(d_freq_aux) ||
                std::abs(d_fine_freq) > max_fine_freq) {
                std::fprintf(stderr,
                             "FREQ_LOST: fine_freq=%.5f, freq_aux=%.5f, "
                             "integer_offset=%d, cp_quality=%.3f\n",
                             d_fine_freq, d_freq_aux, d_freq_offset, d_cp_quality);
                std::fflush(stderr);
                send_sync_start(d_to_output);
                reset_synchronization();
                d_consumed += required_for_interpolation;
                consume_each(d_consumed);
                return d_to_output;
            }
        } else {
        		// Synchronization has been lost.
        		if (d_initial_acquired) {
            std::fprintf(stderr,
                         "CP_LOST: sample=%" PRIu64 ", power=%.1f dBFS, "
                         "quality=%.3f, reason=%s\n",
                         input_base + d_consumed, power_db, d_cp_quality,
                         d_diag_cp_rejection.reason);
            if (d_diagnostics) {
                // Capture both tracking results before resetting the receiver.
                const float wide_quality = d_cp_quality;
                const cp_rejection_diagnostic wide_cp_rejection = d_diag_cp_rejection;
                const int expected_cp = d_cp_start;
                const int available = ninput_items[0] - d_consumed;
                const cp_probe_result probe = probe_cp_correlation(
                    &in[d_consumed], available, d_fft_length, d_cp_length, expected_cp, d_rho);
    
                double recent_min = 0.0;
                double recent_avg = 0.0;
                if (d_diag_recent_count > 0) {
                    recent_min = 1.0;
                    for (std::size_t j = 0; j < d_diag_recent_count; ++j) {
                        recent_min = std::min(recent_min,
                                              static_cast<double>(d_diag_recent_qualities[j]));
                        recent_avg += d_diag_recent_qualities[j];
                    }
                    recent_avg /= d_diag_recent_count;
                }
    
                std::fprintf(stderr, "CP_DIAG: window_start=%" PRIu64
                                     ", power=%.1f dB, quality=%.3f, cp_start=%d"
                                     ", q_narrow=%.3f"
                                     ", last_good_sample=%" PRIu64
                                     ", since_last_good=%" PRIu64
                                     ", last_good_quality=%.3f, recent_min=%.3f"
                                     ", recent_avg=%.3f, last_good_power=%.1f dB"
                                     ", samp_inc=%.8f, samp_phase=%.5f, delta=%.8f"
                                     ", coarse_freq=%.5f, fine_freq=%.5f\n",
                             input_base + d_consumed, power_db, wide_quality, expected_cp,
                             narrow_cp_quality, d_diag_last_good_cp_sample,
                             d_diag_last_good_valid && input_base + d_consumed >=
                                     d_diag_last_good_cp_sample
                                 ? input_base + d_consumed - d_diag_last_good_cp_sample : 0,
                             d_diag_last_good_quality, recent_min, recent_avg,
                             d_diag_last_good_power_db, d_samp_inc, d_samp_phase, d_est_delta,
                             d_coarse_freq, d_fine_freq);
                std::fprintf(stderr,
                             "CP_REJECT: narrow=%s, q=%.3f, ml=%.6g, threshold=%.6g"
                             ", peak=%d/%d; wide=%s, q=%.3f, ml=%.6g, threshold=%.6g"
                             ", peak=%d/%d\n",
                             narrow_cp_rejection.reason, narrow_cp_rejection.quality,
                             narrow_cp_rejection.peak_metric, narrow_cp_rejection.threshold,
                             narrow_cp_rejection.peak_index, narrow_cp_rejection.search_size,
                             wide_cp_rejection.reason, wide_cp_rejection.quality,
                             wide_cp_rejection.peak_metric, wide_cp_rejection.threshold,
                             wide_cp_rejection.peak_index, wide_cp_rejection.search_size);
                std::fprintf(stderr, "CP_PROBE: positions=%d/%d, expected=%d"
                                     ", q_expected=%.3f, q8=%.3f@%d, q16=%.3f@%d"
                                     ", q_global=%.3f@%d, q_ml=%.3f@%d"
                                     ", early_db=%.1f, late_db=%.1f\n",
                             probe.positions, probe.full_positions, expected_cp,
                             probe.expected_quality, probe.best_8_quality, probe.best_8_pos,
                             probe.best_16_quality, probe.best_16_pos,
                             probe.best_quality, probe.best_quality_pos,
                             probe.metric_peak_quality, probe.best_metric_pos,
                             10.0 * std::log10(std::max(probe.expected_early_power, 1.0e-20)),
                             10.0 * std::log10(std::max(probe.expected_late_power, 1.0e-20)));
                report_cp_input_power(&in[d_consumed], available,
                                      2 * d_fft_length + d_cp_length);
                std::fflush(stderr);
            }
            d_diag_await_reacquisition = true;
            d_diag_recent_count = 0;
            d_diag_recent_next = 0;
		            //  Send sync_start downstream
		            send_sync_start(d_to_output);
		            reset_synchronization();
        		}
        		
            // Consume 1/8 of the data and restart the CP search.
            d_consumed += (d_cp_length + d_fft_length) / 8;
            
            // Tell the runtime system how many input items we have consumed on
            // each input stream.
            consume_each(d_consumed);
                    return (d_to_output);
        }
        d_consumed += required_for_interpolation;

        const int symbol_length = d_fft_length + d_cp_length;
        int delta_pos = required_for_interpolation - symbol_length;
        d_cp_moved = delta_pos != 0;

        // Keep a confirmed CP peak away from the hard boundaries of the search
        // buffer. At the right boundary, the peak detector rejects even a
        // perfect CP correlation as an edge peak. Shifting the input origin and
        // the saved CP index equally preserves the absolute symbol position.
        const int centered_cp_position = symbol_length - 1 + d_fft_length / 2;
        const int recenter_shift = d_cp_start - centered_cp_position;
        if (d_initial_acquired && std::abs(recenter_shift) > d_fft_length / 8) {
            const int shifted_consumption = d_consumed + recenter_shift;
            if (shifted_consumption >= 0 && shifted_consumption <= ninput_items[0]) {
                d_consumed = shifted_consumption;
                d_cp_start = centered_cp_position;
                d_cp_moved = true;
                consume_each(d_consumed);
                return d_to_output;
            }
        }
    }

    consume_each(d_consumed);

    return d_to_output;
}

} /* namespace dl5eu */
} /* namespace gr */
