/* -*- c++ -*- */
/*
 * Copyright 2025 Ralf Gorholt.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_DL5EU_DVBT_OFDM_SYNCHRONIZATION_H
#define INCLUDED_DL5EU_DVBT_OFDM_SYNCHRONIZATION_H

#include <gnuradio/block.h>
#include <gnuradio/dl5eu/api.h>
#include <gnuradio/dtv/dvb_config.h>
#include <gnuradio/dtv/dvbt_config.h>

namespace gr {
namespace dl5eu {

/*!
 * \brief OFDM symbol recognition and synchronization.
 * \ingroup dl5eu
 *
 * Data input format: \n
 * complex(real(float), imag(float)). \n
 * Data output format: \n
 * complex(real(float), imag(float)).
 */
class DL5EU_API dvbt_ofdm_synchronization : virtual public gr::block
{
public:
    typedef std::shared_ptr<dvbt_ofdm_synchronization> sptr;

    /*!
     * \brief Creates an OFDM Synchronization block for DVB-T.
     *
     * \param transmission_mode the transmission mode (2K or 8K). \n
     * \param guard_interval the guard interval (1/32, 1/16, 1/8 or 1/4). \n
     * \param snr the SNR (default = 10.0). \n
     * \param interpolate whether interpolation is used or not. \n
     * \param sample_rate nominal complex input sample rate after resampling, fixed at construction (Hz). \n
     * \param squelch_db digital IQ power threshold (dBFS), or -200 to disable. \n
     * \param diagnostics enable verbose synchronization diagnostics. \n
     */
    static sptr make(dvbt_transmission_mode_t transmission_mode,
                     dvb_guardinterval_t guard_interval,
                     float snr,
                     bool interpolate,
                     double sample_rate = 8.0e6 / 7.0,
                     float squelch_db = -60.0f,
                     bool diagnostics = false);

    virtual void request_resynchronization() = 0;

    virtual void set_squelch_threshold(float threshold_db) = 0;
    virtual float get_squelch_threshold() const = 0;
};

} // namespace dl5eu
} // namespace gr

#endif /* INCLUDED_DL5EU_DVBT_OFDM_SYNCHRONIZATION_H */
