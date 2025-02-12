/* -*- c++ -*- */
/*
 * Copyright 2015,2016 Free Software Foundation, Inc.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifndef INCLUDED_DL5EU_DVBT_CONVOLUTIONAL_DEINTERLEAVER_H
#define INCLUDED_DL5EU_DVBT_CONVOLUTIONAL_DEINTERLEAVER_H

#include <gnuradio/block.h>
#include <gnuradio/dl5eu/api.h>

namespace gr {
namespace dl5eu {

/*!
 * \brief Convolutional deinterleaver.
 * \ingroup dtv
 *
 * ETSI EN 300 744 Clause 4.3.1 \n
 * Forney (Ramsey type III) convolutional deinterleaver. \n
 * Data input: Stream of 1 byte elements. \n
 * Data output: Blocks of I bytes size.
 */
class DL5EU_API dvbt_convolutional_deinterleaver : virtual public gr::block
{
public:
    typedef std::shared_ptr<dvbt_convolutional_deinterleaver> sptr;

    /*!
     * \brief Create a DVB-T convolutional deinterleaver.
     *
     * \param nsize number of blocks to process. \n
     * \param I size of a block. \n
     * \param M depth length for each element in shift registers.
     */
    static sptr make(int nsize, int I, int M);
};

} // namespace dl5eu
} // namespace gr

#endif /* INCLUDED_DL5EU_DVBT_CONVOLUTIONAL_DEINTERLEAVER_H */
