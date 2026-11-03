/* -*- c++ -*- */
/*
 * Copyright 2025 Ralf Gorholt.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_DL5EU_DVBT_ENERGY_DESCRAMBLE_IMPL_H
#define INCLUDED_DL5EU_DVBT_ENERGY_DESCRAMBLE_IMPL_H

#include <gnuradio/dl5eu/dvbt_energy_descramble.h>

namespace gr {
namespace dl5eu {

class dvbt_energy_descramble_impl : public dvbt_energy_descramble
{
private:
    static const int d_nblocks;
    static const int d_bsize;
    static const int d_SYNC;
    static const int d_NSYNC;
    static const int d_MUX_PKT;

    // Register for PRBS
    int d_reg;

    // Index
    int d_index;
    // Search interval
    int d_search;

    void init_prbs();
    int clock_prbs(int clocks);

public:
    dvbt_energy_descramble_impl(int nblocks);
    ~dvbt_energy_descramble_impl() override;

    void forecast(int noutput_items, gr_vector_int& ninput_items_required) override;

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items) override;
};

} // namespace dl5eu
} // namespace gr

#endif /* INCLUDED_DL5EU_DVBT_ENERGY_DESCRAMBLE_IMPL_H */
