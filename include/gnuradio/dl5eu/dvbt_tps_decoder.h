/* -*- c++ -*- */
/*
 * Copyright 2025 Ralf Gorholt.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_DL5EU_DVBT_TPS_DECODER_H
#define INCLUDED_DL5EU_DVBT_TPS_DECODER_H

#include <gnuradio/block.h>
#include <gnuradio/dl5eu/api.h>
#include <gnuradio/dtv/dvbt_config.h>

namespace gr {
namespace dl5eu {

/*!
 * \brief Given the active carriers, this block decodes the TPS data and outputs
 * those carriers that transport the "actual" TV data (i.e. without pilots).
 * \ingroup dl5eu
 *
 * Data input format: \n
 * complex(real(float), imag(float)). \n
 * Data output format: \n
 * complex(real(float), imag(float)).
 */
class DL5EU_API dvbt_tps_decoder : virtual public gr::block
{
public:
    typedef std::shared_ptr<dvbt_tps_decoder> sptr;

    /*!
     * \brief Creates a TPS Decoder block for DVB-T.
     *
     * \param transmission_mode the transmission mode (2K or 8K). \n
     * \param print_tps_data whether the decoded TPS data are printed on the console or
     * not. \n
     */
    static sptr make(dvbt_transmission_mode_t transmission_mode, bool print_tps_data);
};

} // namespace dl5eu
} // namespace gr

#endif /* INCLUDED_DL5EU_DVBT_TPS_DECODER_H */
