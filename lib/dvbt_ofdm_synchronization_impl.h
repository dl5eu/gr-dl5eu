/* -*- c++ -*- */
/*
 * Copyright 2025 Ralf Gorholt.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * ==============================================================
 *  This block is based on the block "ofdm_synchronization" that
 *  Federico La Rocca and others have developped for ISDB-T.
 * ==============================================================
 */

#ifndef INCLUDED_DL5EU_DVBT_OFDM_SYNCHRONIZATION_IMPL_H
#define INCLUDED_DL5EU_DVBT_OFDM_SYNCHRONIZATION_IMPL_H

#include "dvbt_configure.h"
#include "dvbt_pilot_gen.h"
#include <gnuradio/dl5eu/dvbt_ofdm_synchronization.h>
#include <gnuradio/fft/fft.h>
#include <gnuradio/filter/mmse_fir_interpolator_cc.h>

namespace gr {
namespace dl5eu {

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

    // Phase correction
    double d_phaseinc;
    double d_nextphaseinc;
    int d_nextpos;
    float d_phase;

    bool d_initial_acquired;

    int d_consumed;
    int d_to_output;

    // FFT part
    volk::vector<gr_complex> d_postfft;
    gr::fft::fft_complex_fwd d_fft_calculator;

    // Zero padding to the left
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
    // These are some variables that I will use to increase performance.
    // The interpolation coefficients I use for linear interpolation.
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

    void advance_freq_loop(float error);

    void advance_delta_loop(float error);

    int interpolate_input(const gr_complex* in, gr_complex* out);

    /*!
     * \brief Estimates post-FFT synchronization parameters.
     */
    void estimate_fine_synchro(gr_complex* current_channel, gr_complex* previous_channel);

    /*!
     * Signals downstream the symbol index and (if necessary) resynching.
     */
    void send_symbol_index_and_resync(int current_offset);

    /*!
     * \brief Calculates the channel taps based on pilots. This is the linear very simple
     * implementation.
     */
    void linearly_estimate_channel_taps(int current_symbol, gr_complex* channel_gain);

    /*!
     * \brief Calculates the channel taps at the SPs, given the input complex baseband
     * signal and a symbol number (between 0 and 3). The method that integer frequency
     * offset has been corrected.
     */
    void calculate_channel_taps_sp(const gr_complex* in,
                                   int current_symbol,
                                   gr_complex* channel_gain);

    /*!
     * \brief Estimates the current symbol. Necessary for equalization.
     */
    int estimate_symbol_index(const gr_complex* in);

    /*!
     * This method constructs the pseudo-random sequence wk used by the standard.
     */
    void generate_prbs();

    /*!
     * \brief This method is responsible for the integer frequency error estimation.
     */
    int estimate_integer_freq_offset(const gr_complex* in);

    /*!
     * \brief It calculate the FFT of what is saved on the d_fft_calculator.get_inbuf()
     * and saves it on out. Plus, it performs an fft shift. Note that the fft_size is
     * assumed given by d_fft_length.
     */
    void calculate_fft(gr_complex* out);

    /*!
     * \brief Calculates the likelihood function and outputs the position of its maximum.
     *
     * Given the input, it calculates the resulting likelihood function between indices
     * lookup_stop and lookup_start. It returns the beginning of the CP (in cp_pos), and
     * the value of epsilon (see the paper). to_consume and to_out was used as indicators
     * of whether the peak was correctly found or not. Now the return value is used
     * (either true or false).
     */
    bool ml_sync(const gr_complex* in,
                 int lookup_start,
                 int lookup_stop,
                 int* cp_pos,
                 float* peak_epsilon);

    /*!
     * \brief Initializes the parameters used in the peak_detect_process.
     *
     * \param threshold_factor_rise The algorithm keeps an average of minimum and maximum
     * value. A peak is considered valid when it's bigger than avg_max -
     * threshold_factor_rise(avg_max-avg_min).
     * \param alpha The parameter is used to update both the average maximum and minimum
     * (exponential filter, or single-root iir).
     */
    void init_peak_detection(float threshold_factor_rise, float alpha);

    /*!
     * \brief Given datain and its length, the method returns the peak position and its
     * value.
     */
    bool detect_peak_position(const float* data, const int data_size, int* peak_pos);

    /*!
     * \brief Sends a tag downstream that signals that acquisition was successfully
     * performed (or that we lost synchronization and we have regained it).
     */
    void send_sync_start();

    /*!
     * \brief Using the vector derot calculated by ml_sync (an exponential modulated with
     * minus the estimated frequency error), this method simply multiplies it by the input
     * and saves it in the output.
     */
    void derotate(const gr_complex* in, gr_complex* out);

public:
    dvbt_ofdm_synchronization_impl(dvbt_transmission_mode_t transmission_mode,
                                   dvb_guardinterval_t guard_interval,
                                   float snr,
                                   bool interpolate);
    ~dvbt_ofdm_synchronization_impl();

    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);
};

} // namespace dl5eu
} // namespace gr

#endif /* INCLUDED_DL5EU_DVBT_OFDM_SYNCHRONIZATION_IMPL_H */
