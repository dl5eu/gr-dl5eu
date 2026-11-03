/* -*- c++ -*- */
/*
 * Copyright 2025 Ralf Gorholt.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_DL5EU_DVBT_REED_SOLOMON_DEC_H
#define INCLUDED_DL5EU_DVBT_REED_SOLOMON_DEC_H

#include <gnuradio/block.h>
#include <gnuradio/dl5eu/api.h>

namespace gr {
namespace dl5eu {

/*!
 * \brief Reed Solomon decoder.
 * \ingroup dtv
 *
 * ETSI EN 300 744 Clause 4.3.2 \n
 * RS(N=204,K=239,T=8).
 */
class DL5EU_API dvbt_reed_solomon_dec : virtual public block
{
public:
    typedef std::shared_ptr<dvbt_reed_solomon_dec> sptr;

    /*!
     * \brief Create a Reed Solomon decoder.
     *
     * \param p characteristic of GF(p^m).
     * \param m we use GF(p^m).
     * \param gfpoly Generator Polynomial.
     * \param n length of codeword of RS coder.
     * \param k length of information sequence of RS decoder.
     * \param t number of corrected errors.
     * \param s shortened length.
     * \param blocks number of blocks to process at once.
     * \param diagnostics enable console diagnostic output.
     */
    static sptr make(int p,
                     int m,
                     int gfpoly,
                     int n,
                     int k,
                     int t,
                     int s,
                     int blocks,
                     bool diagnostics = false);
};

} // namespace dl5eu
} // namespace gr

#endif /* INCLUDED_DL5EU_DVBT_REED_SOLOMON_DEC_H */
