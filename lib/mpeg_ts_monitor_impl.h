/* -*- c++ -*- */
/*
 * Copyright 2026 Ralf Gorholt.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_DL5EU_MPEG_TS_MONITOR_IMPL_H
#define INCLUDED_DL5EU_MPEG_TS_MONITOR_IMPL_H

#include <gnuradio/dl5eu/mpeg_ts_monitor.h>

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace gr {
namespace dl5eu {

/*!
 * @brief Observe MPEG-TS packet integrity without modifying or gating the byte stream.
 *
 * Packet analysis keeps its own buffered alignment; output is always a byte-for-byte
 * copy of input, including bytes rejected by the analyzer. A separate timer thread
 * reports even during upstream stalls. This detects packet/header/continuity faults,
 * not arbitrary payload corruption or a complete transport-stream compliance check.
 */
class mpeg_ts_monitor_impl : public mpeg_ts_monitor
{
private:
    static constexpr std::size_t packet_size = 188;
    static constexpr std::size_t pid_count = 8192;
    static constexpr std::size_t max_events_per_interval = 32;

    using steady_clock = std::chrono::steady_clock;

    // Last trusted continuity state and complete packet for duplicate detection.
    struct pid_state {
        bool valid{ false };
        std::uint8_t continuity_counter{ 0 };
        unsigned int duplicate_count{ 0 };
        std::array<std::uint8_t, packet_size> previous_packet{};
    };

    // Lifetime totals; interval values are computed by subtracting snapshots.
    struct counters {
        std::uint64_t bytes{ 0 };
        std::uint64_t packets{ 0 };
        std::uint64_t sync_losses{ 0 };
        std::uint64_t discarded_bytes{ 0 };
        std::uint64_t tei_errors{ 0 };
        std::uint64_t cc_errors{ 0 };
        std::uint64_t header_errors{ 0 };
        std::uint64_t duplicate_packets{ 0 };
    };

    struct event {
        std::chrono::system_clock::time_point timestamp;
        std::string name;
        std::string detail;
        std::uint64_t byte_offset{ 0 };
        std::uint64_t packet_number{ 0 };
        int pid{ -1 };
    };

    struct report {
        std::chrono::system_clock::time_point timestamp;
        steady_clock::time_point steady_timestamp;
        counters totals;
        counters previous;
        std::vector<event> events;
        std::uint64_t omitted_events{ 0 };
        bool locked{ false };
        bool final{ false };
        double seconds{ 0.0 };
    };

    const std::chrono::milliseconds d_log_interval;
    const bool d_console_events;
    const bool d_console_statistics;
    const steady_clock::time_point d_start_time;
    std::ofstream d_log_file;
    std::thread d_log_thread;
    std::mutex d_mutex;
    std::condition_variable d_condition;
    bool d_stopping{ false };

    counters d_totals;
    counters d_previous;
    steady_clock::time_point d_previous_time;
    std::vector<event> d_events;
    std::uint64_t d_omitted_events{ 0 };

    std::array<pid_state, pid_count> d_pid_states{};
    // Analyzer-only history; the forwarding path never reads bytes back from it.
    std::deque<std::uint8_t> d_pending;
    std::uint64_t d_pending_offset{ 0 };
    bool d_locked{ false };
    bool d_ever_locked{ false };

    /*!
     * @brief Append one byte to the analyzer and recover 188-byte packet alignment.
     * @param value Next stream byte; d_totals.bytes tracks its absolute input position.
     *
     * Three sync bytes spaced by 188 bytes establish alignment. Once locked, complete
     * packets are inspected; a missing boundary sync byte restarts the sliding search.
     * Analysis discards never remove bytes from the actual output. Caller holds d_mutex.
     */
    void process_byte(std::uint8_t value);
    /*!
     * @brief Check a sync-aligned packet and update PID-specific continuity history.
     * @param packet Complete 188-byte packet beginning with 0x47.
     * @param byte_offset Absolute zero-based input offset of its sync byte.
     *
     * Validate adaptation lengths and TEI before trusting continuity. Payload advances
     * the four-bit counter; adaptation-only packets do not. Discontinuity flags reset
     * the expectation, null packets are excluded and up to two identical payload
     * repeats are accepted by this implementation. Caller holds d_mutex.
     */
    void process_packet(const std::array<std::uint8_t, packet_size>& packet,
                        std::uint64_t byte_offset);
    /*!
     * @brief Queue a timestamped event without allowing an unbounded diagnostic backlog.
     * @param name Event label.
     * @param detail Human-readable context.
     * @param byte_offset Absolute zero-based input byte offset.
     * @param pid Affected PID, or -1 for events not tied to a PID.
     *
     * Keep at most 32 event details per reporting interval; omitted events are counted
     * and cumulative error totals still advance. Caller holds d_mutex.
     */
    void add_event(const std::string& name,
                   const std::string& detail,
                   std::uint64_t byte_offset,
                   int pid = -1);
    /*!
     * @brief Snapshot cumulative counters and detach interval events for logging.
     * @param final Mark a shutdown report rather than a periodic report.
     * @return Consistent snapshot with current and previous totals and elapsed seconds.
     *
     * Advance the interval baseline without clearing cumulative counters. Caller holds
     * d_mutex; detached data can be formatted after releasing the processing lock.
     */
    report take_report(bool final);
    /*!
     * @brief Wait on steady-clock intervals and report independently of stream callbacks.
     *
     * Hold d_mutex only to wait or take a snapshot; release it during file/console IO.
     * Shutdown notification ends the wait promptly and produces one final report.
     */
    void log_loop();
    /*!
     * @brief Format a detached report for CSV and the enabled console outputs.
     * @param value Snapshot returned by take_report().
     *
     * Cumulative totals and interval differences have distinct meanings. Bitrate counts
     * all received bytes, including bytes rejected by analysis, per elapsed wall time
     * measured with a steady clock. Only the logging thread writes reports.
     */
    void write_report(const report& value);

public:
    /*!
     * @brief Configure reporting destinations and start the independent timer thread.
     * @param log_interval_ms Positive reporting period in milliseconds.
     * @param log_filename CSV filename; empty disables file output, otherwise truncates it.
     * @param console_events Enable detailed console event output.
     * @param console_statistics Enable periodic console counter/rate output.
     * @throws std::invalid_argument If the reporting period is not positive.
     * @throws std::runtime_error If the requested CSV file cannot be opened.
     *
     * Wall-clock timestamps label events; steady-clock durations determine rates.
     */
    mpeg_ts_monitor_impl(int log_interval_ms,
                         const std::string& log_filename,
                         bool console_events,
                         bool console_statistics);
    /*!
     * @brief Wake and join the logger so its final snapshot is written before destruction.
     *
     * The join prevents the background thread from accessing destroyed member storage.
     */
    ~mpeg_ts_monitor_impl() override;

    /*!
     * @brief Forward bytes unchanged and inspect a separate packet-analysis history.
     * @param noutput_items Number of bytes to copy and analyze.
     * @param input_items Input byte-stream buffers.
     * @param output_items Output byte-stream buffers.
     * @return noutput_items; sync_block handles equal input consumption automatically.
     *
     * The mutex serializes analyzer updates with report snapshots. Packet acquisition
     * latency affects diagnostic visibility only, never output alignment or rate.
     */
    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items) override;
};

} // namespace dl5eu
} // namespace gr

#endif /* INCLUDED_DL5EU_MPEG_TS_MONITOR_IMPL_H */
