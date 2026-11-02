/* -*- c++ -*- */
/*
 * Copyright 2026 Ralf Gorholt.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "mpeg_ts_monitor_impl.h"

#include <gnuradio/io_signature.h>

#include <algorithm>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace gr {
namespace dl5eu {

namespace {

/*!
 * @brief Format a local wall-clock timestamp with millisecond precision.
 * @param timestamp Wall-clock event time.
 * @return Local calendar timestamp with millisecond precision; not a duration clock.
 */
std::string local_timestamp(std::chrono::system_clock::time_point timestamp)
{
    const auto seconds = std::chrono::system_clock::to_time_t(timestamp);
    std::tm local{};
#ifdef _WIN32
    localtime_s(&local, &seconds);
#else
    localtime_r(&seconds, &local);
#endif

    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        timestamp.time_since_epoch()).count() % 1000;

    std::ostringstream result;
    result << std::put_time(&local, "%Y-%m-%d %H:%M:%S") << '.' << std::setfill('0')
           << std::setw(3) << milliseconds;
    return result.str();
}

/*!
 * @brief Quote a CSV field and escape embedded quotes.
 * @param value Unescaped CSV field content.
 * @return Quoted field with embedded double quotes doubled.
 */
std::string csv_field(const std::string& value)
{
    std::string result = "\"";
    for (char character : value) {
        if (character == '"') {
            result += '"';
        }
        result += character;
    }
    result += '"';
    return result;
}

} // namespace

/*! @brief Creates a byte-transparent monitor for 188-byte MPEG transport packets. */
mpeg_ts_monitor::sptr mpeg_ts_monitor::make(int log_interval_ms,
                                            const std::string& log_filename,
                                            bool console_events,
                                            bool console_statistics)
{
    return gnuradio::make_block_sptr<mpeg_ts_monitor_impl>(
        log_interval_ms, log_filename, console_events, console_statistics);
}

/*! @brief Configures the packet monitor and starts its independent statistics timer. */
mpeg_ts_monitor_impl::mpeg_ts_monitor_impl(int log_interval_ms,
                                           const std::string& log_filename,
                                           bool console_events,
                                           bool console_statistics)
    : gr::sync_block("mpeg_ts_monitor",
                     gr::io_signature::make(1, 1, sizeof(std::uint8_t)),
                     gr::io_signature::make(1, 1, sizeof(std::uint8_t)))
    , d_log_interval(log_interval_ms)
    , d_console_events(console_events)
    , d_console_statistics(console_statistics)
    , d_start_time(steady_clock::now())
    , d_previous_time(d_start_time)
{
    if (log_interval_ms <= 0) {
        throw std::invalid_argument("mpeg_ts_monitor: log_interval_ms must be positive");
    }

    if (!log_filename.empty()) {
        d_log_file.open(log_filename, std::ios::out | std::ios::trunc);
        if (!d_log_file.is_open()) {
            throw std::runtime_error("mpeg_ts_monitor: cannot open log file: " + log_filename);
        }
        d_log_file << "timestamp,type,elapsed_ms,total_bytes,byte_offset,packet_number,"
                      "packets_interval,bitrate_bps,sync_losses,discarded_bytes,tei_errors,"
                      "cc_errors,header_errors,duplicates,pid,detail\n";
        d_log_file.flush();
    }

    d_log_thread = std::thread(&mpeg_ts_monitor_impl::log_loop, this);
}

/*! @brief Stops the statistics timer, writes its final report and closes the log file. */
mpeg_ts_monitor_impl::~mpeg_ts_monitor_impl()
{
    {
        std::lock_guard<std::mutex> lock(d_mutex);
        d_stopping = true;
    }
    d_condition.notify_one();
    if (d_log_thread.joinable()) {
        d_log_thread.join();
    }
}

/*! @brief Records a bounded number of timestamped diagnostic events per interval. */
void mpeg_ts_monitor_impl::add_event(const std::string& name,
                                     const std::string& detail,
                                     std::uint64_t byte_offset,
                                     int pid)
{
    if (d_events.size() >= max_events_per_interval) {
        ++d_omitted_events;
        return;
    }
    d_events.push_back({ std::chrono::system_clock::now(),
                         name, detail, byte_offset, d_totals.packets, pid });
}

/*! @brief Checks TS headers, TEI and continuity counters without changing input data. */
void mpeg_ts_monitor_impl::process_packet(
    const std::array<std::uint8_t, packet_size>& packet,
    std::uint64_t byte_offset)
{
    ++d_totals.packets;
    const int pid = ((packet[1] & 0x1f) << 8) | packet[2];
    const bool tei = (packet[1] & 0x80) != 0;
    const int adaptation_control = (packet[3] >> 4) & 0x03;
    const std::uint8_t continuity_counter = packet[3] & 0x0f;

    if (adaptation_control == 0 ||
        (adaptation_control >= 2 && packet[4] > 183) ||
        (adaptation_control == 3 && packet[4] > 182)) {
        ++d_totals.header_errors;
        add_event("HEADER_ERROR", "invalid adaptation_field_control or length",
                  byte_offset, pid);
        d_pid_states[pid].valid = false;
        return;
    }

    const bool has_payload = (adaptation_control & 0x01) != 0;
    const bool has_adaptation = (adaptation_control & 0x02) != 0;
    const bool discontinuity = has_adaptation && packet[4] > 0 && (packet[5] & 0x80);

    if (tei) {
        ++d_totals.tei_errors;
        add_event("TEI", "transport_error_indicator=1", byte_offset, pid);
        d_pid_states[pid].valid = false;
        return;
    }

    // Null packets have no meaningful PID-specific continuity sequence.
    if (pid == 0x1fff) {
        return;
    }

    pid_state& state = d_pid_states[pid];
    if (state.valid && !discontinuity) {
        const std::uint8_t expected =
            (state.continuity_counter + (has_payload ? 1 : 0)) & 0x0f;

        // This monitor tolerates up to two repeats identical in all 188 bytes.
        // Treat this as the analyzer policy, not a complete duplicate-packet validator.
        const bool duplicate = has_payload && continuity_counter == state.continuity_counter &&
            std::memcmp(packet.data(), state.previous_packet.data(), packet_size) == 0 &&
            state.duplicate_count < 2;

        if (duplicate) {
            ++state.duplicate_count;
            ++d_totals.duplicate_packets;
            return;
        }

        if (continuity_counter != expected) {
            ++d_totals.cc_errors;
            std::ostringstream message;
            message << "expected=" << unsigned(expected)
                    << " received=" << unsigned(continuity_counter);
            add_event("CC_ERROR", message.str(), byte_offset, pid);
        }
    }

    state.valid = true;
    state.continuity_counter = continuity_counter;
    state.duplicate_count = 0;
    state.previous_packet = packet;
}

/*! @brief Recovers 188-byte packet alignment from an arbitrary input byte offset. */
void mpeg_ts_monitor_impl::process_byte(std::uint8_t value)
{
    d_pending.push_back(value);
    ++d_totals.bytes;

    while (true) {
        if (!d_locked) {
            // Three 0x47 bytes with exactly 188 bytes spacing are required for a lock.
            if (d_pending.size() < 2 * packet_size + 1) {
                return;
            }
            if (d_pending[0] != 0x47 || d_pending[packet_size] != 0x47 ||
                d_pending[2 * packet_size] != 0x47) {
                d_pending.pop_front();
                ++d_pending_offset;
                ++d_totals.discarded_bytes;
                continue;
            }
            d_locked = true;
            add_event(d_ever_locked ? "SYNC_RECOVERED" : "SYNC_ACQUIRED", "",
                      d_pending_offset);
            d_ever_locked = true;
        }

        if (d_pending.size() < packet_size) {
            return;
        }
        if (d_pending.front() != 0x47) {
            ++d_totals.sync_losses;
            add_event("SYNC_LOST", "expected 0x47 at packet boundary", d_pending_offset);
            d_locked = false;
            d_pending.pop_front();
            ++d_pending_offset;
            ++d_totals.discarded_bytes;
            continue;
        }

        std::array<std::uint8_t, packet_size> packet{};
        for (std::size_t i = 0; i < packet_size; ++i) {
            packet[i] = d_pending.front();
            d_pending.pop_front();
        }
        const std::uint64_t offset = d_pending_offset;
        d_pending_offset += packet_size;
        process_packet(packet, offset);
    }
}

/*! @brief Copies each byte exactly once and inspects packets with no stream-rate changes. */
int mpeg_ts_monitor_impl::work(int noutput_items,
                               gr_vector_const_void_star& input_items,
                               gr_vector_void_star& output_items)
{
    const auto* input = static_cast<const std::uint8_t*>(input_items[0]);
    auto* output = static_cast<std::uint8_t*>(output_items[0]);
    std::memcpy(output, input, static_cast<std::size_t>(noutput_items));

    std::lock_guard<std::mutex> lock(d_mutex);
    for (int i = 0; i < noutput_items; ++i) {
        process_byte(input[i]);
    }
    return noutput_items;
}

/*! @brief Detach interval events and advance reporting baselines under d_mutex. */
mpeg_ts_monitor_impl::report mpeg_ts_monitor_impl::take_report(bool final)
{
    report result;
    result.timestamp = std::chrono::system_clock::now();
    result.steady_timestamp = steady_clock::now();
    result.totals = d_totals;
    result.previous = d_previous;
    result.seconds = std::chrono::duration<double>(result.steady_timestamp - d_previous_time)
                         .count();
    result.locked = d_locked;
    result.final = final;
    result.events.swap(d_events);
    result.omitted_events = d_omitted_events;
    d_omitted_events = 0;
    d_previous = d_totals;
    d_previous_time = result.steady_timestamp;
    return result;
}

/*! @brief Writes periodic reports to CSV and optionally to the console. */
void mpeg_ts_monitor_impl::write_report(const report& value)
{
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        value.steady_timestamp - d_start_time).count();
    const auto bytes_interval = value.totals.bytes - value.previous.bytes;
    const auto packets_interval = value.totals.packets - value.previous.packets;
    const auto bitrate = value.seconds > 0.0 ? bytes_interval * 8.0 / value.seconds : 0.0;
    const auto timestamp = local_timestamp(value.timestamp);

    if (d_console_statistics) {
        std::cout << "TS_MONITOR: " << timestamp
                  << " packets=" << value.totals.packets
                  << " rate=" << std::fixed << std::setprecision(0)
                  << (value.seconds > 0.0 ? packets_interval / value.seconds : 0.0)
                  << " pkt/s bitrate=" << bitrate
                  << " bps sync_losses=" << value.totals.sync_losses
                  << " tei=" << value.totals.tei_errors
                  << " cc=" << value.totals.cc_errors
                  << " header=" << value.totals.header_errors
                  << " lock=" << (value.locked ? 1 : 0)
                  << (value.final ? " final" : "") << std::endl;
    }

    if (!d_log_file.is_open()) {
        if (d_console_events) {
            for (const auto& entry : value.events) {
                std::cout << "TS_EVENT: " << local_timestamp(entry.timestamp) << ' '
                          << entry.name << " offset=" << entry.byte_offset
                          << " pid=" << entry.pid << ' ' << entry.detail << std::endl;
            }
            if (value.omitted_events != 0) {
                std::cout << "TS_MONITOR: " << value.omitted_events
                          << " additional events suppressed" << std::endl;
            }
        }
        return;
    }

    d_log_file << timestamp << "," << (value.final ? "FINAL" : "STAT") << ','
               << elapsed_ms << ',' << value.totals.bytes << ',' << value.totals.bytes << ','
               << value.totals.packets << ',' << packets_interval << ','
               << std::fixed << std::setprecision(0) << bitrate << ','
               << value.totals.sync_losses << ',' << value.totals.discarded_bytes << ','
               << value.totals.tei_errors << ',' << value.totals.cc_errors << ','
               << value.totals.header_errors << ',' << value.totals.duplicate_packets
               << ",," << csv_field(value.locked ? "locked" : "searching") << '\n';

    for (const auto& entry : value.events) {
        d_log_file << local_timestamp(entry.timestamp) << ",EVENT,,," << entry.byte_offset
                   << ',' << entry.packet_number << ",,,,,,,,,";
        if (entry.pid >= 0) {
            d_log_file << entry.pid;
        }
        d_log_file << ',' << csv_field(entry.name + " " + entry.detail) << '\n';
        if (d_console_events) {
            std::cout << "TS_EVENT: " << local_timestamp(entry.timestamp) << ' '
                      << entry.name << " offset=" << entry.byte_offset
                      << " pid=" << entry.pid << ' ' << entry.detail << std::endl;
        }
    }
    if (value.omitted_events != 0) {
        d_log_file << timestamp << ",OMITTED,,,,,,,,,,,,,,"
                   << csv_field(std::to_string(value.omitted_events) + " events") << '\n';
        if (d_console_events) {
            std::cout << "TS_MONITOR: " << value.omitted_events
                      << " additional events suppressed" << std::endl;
        }
    }
    d_log_file.flush();
}

/*! @brief Report on a separate timer, including upstream stalls and final shutdown. */
void mpeg_ts_monitor_impl::log_loop()
{
    std::unique_lock<std::mutex> lock(d_mutex);
    while (!d_stopping) {
        if (d_condition.wait_for(lock, d_log_interval, [this] { return d_stopping; })) {
            break;
        }
        report value = take_report(false);
        lock.unlock();
        write_report(value);
        lock.lock();
    }
    report value = take_report(true);
    lock.unlock();
    write_report(value);
}

} // namespace dl5eu
} // namespace gr
