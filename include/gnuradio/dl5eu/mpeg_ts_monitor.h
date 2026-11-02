/* -*- c++ -*- */
/*
 * Copyright 2026 Ralf Gorholt.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_DL5EU_MPEG_TS_MONITOR_H
#define INCLUDED_DL5EU_MPEG_TS_MONITOR_H

#include <gnuradio/dl5eu/api.h>
#include <gnuradio/sync_block.h>
#include <memory>
#include <string>

namespace gr {
namespace dl5eu {

class DL5EU_API mpeg_ts_monitor : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<mpeg_ts_monitor> sptr;

    static sptr make(int log_interval_ms = 5000,
                     const std::string& log_filename = "",
                     bool console_events = false,
                     bool console_statistics = false);
};

} // namespace dl5eu
} // namespace gr

#endif /* INCLUDED_DL5EU_MPEG_TS_MONITOR_H */
