#!/usr/bin/env python3
"""Apply a narrowly scoped TPS decoder consume/tag fix without replacing other edits."""

import argparse
from pathlib import Path
import shutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "path",
        nargs="?",
        default="lib/dvbt_tps_decoder_impl.cc",
        help="Path to the current dvbt_tps_decoder_impl.cc",
    )
    args = parser.parse_args()
    path = Path(args.path)
    original = path.read_bytes()
    crlf = b"\r\n" in original
    content = original.decode("utf-8").replace("\r\n", "\n")

    old_block = '''        if (d_frame_end) {
            // Being here means that the previous OFDM symbol constituted the end of a
            // frame. The current symbol is the first symbol of the next frame!
            d_frame_index = (d_frame_index + 1) % d_frames_per_superframe;
            d_symbol_index = 0;
            d_rel_symbol_index = 0;
            const uint64_t offset = this->nitems_written(0) + i;
            // This is a frame start, we signal it downstream
            pmt::pmt_t key = pmt::string_to_symbol("frame_start");
            pmt::pmt_t value = pmt::from_long(d_frame_index);
            pmt::pmt_t srcid = pmt::string_to_symbol(blockid);
            this->add_item_tag(0, offset, key, value, srcid);

            // If "sync_start" and "resync" have been detected we need to re-synchronize.
            if (d_sync_start || d_resync) {
                const uint64_t offset = this->nitems_written(0) + i;
                pmt::pmt_t key = pmt::string_to_symbol("resync");
                pmt::pmt_t value = pmt::from_long(0);
                pmt::pmt_t srcid = pmt::string_to_symbol(blockid);
                this->add_item_tag(0, offset, key, value, srcid);
            }
        } else {'''
    new_block = '''        const bool frame_start = d_frame_end;
        const bool resync_pending = d_sync_start || d_resync;

        if (frame_start) {
            // Being here means that the previous OFDM symbol constituted the end of a
            // frame. The current symbol is the first symbol of the next frame!
            d_frame_index = (d_frame_index + 1) % d_frames_per_superframe;
            d_symbol_index = 0;
            d_rel_symbol_index = 0;
        } else {'''

    old_return = '''                consume_each(1); // Consume the input
                return (0);      // Nothing has been produced'''
    new_return = '''                // Consume all processed input symbols, including this discarded one.
                // Preserve the output symbols already produced in this call.
                consume_each(i + 1);
                return i;'''

    marker = '''        // Send a tag for each OFDM symbol informing about the symbol index.'''
    new_tags = '''        if (frame_start) {
            const uint64_t offset = this->nitems_written(0) + i;
            // This is a frame start, we signal it downstream
            pmt::pmt_t key = pmt::string_to_symbol("frame_start");
            pmt::pmt_t value = pmt::from_long(d_frame_index);
            pmt::pmt_t srcid = pmt::string_to_symbol(blockid);
            this->add_item_tag(0, offset, key, value, srcid);

            // If "sync_start" and "resync" have been detected we need to re-synchronize.
            if (resync_pending) {
                pmt::pmt_t key = pmt::string_to_symbol("resync");
                pmt::pmt_t value = pmt::from_long(0);
                pmt::pmt_t srcid = pmt::string_to_symbol(blockid);
                this->add_item_tag(0, offset, key, value, srcid);
            }
        }

'''

    edits = [
        (old_block, new_block, "move frame_start/resync tags after output decision"),
        (old_return, new_return, "fix input consumption and produced item count"),
        (marker, new_tags + marker, "emit tags only for output symbols"),
    ]
    for old, new, label in edits:
        count = content.count(old)
        if count != 1:
            raise RuntimeError(f"Expected exactly one match for '{label}', found {count}. "
                               "The file was left unchanged.")
        content = content.replace(old, new, 1)

    # Startup must wait for the first valid TPS superframe even if no sync_start tag arrives.
    if content.count("      d_sync_start{ false },") == 1:
        content = content.replace("      d_sync_start{ false },",
                                  "      d_sync_start{ true },", 1)
    elif content.count("      d_sync_start{ true },") != 1:
        raise RuntimeError("Could not locate d_sync_start initializer. File unchanged.")

    edited = content.replace("\n", "\r\n" if crlf else "\n").encode("utf-8")
    backup = path.with_name(path.name + ".bak")
    if backup.exists():
        raise RuntimeError(f"Backup already exists: {backup}. File unchanged.")
    shutil.copy2(path, backup)
    path.write_bytes(edited)
    print(f"Changed: {path}")
    print(f"Backup:  {backup}")
    print("Modified: init sync_start=true; consume_each(i+1), return i; delayed frame/resync tags")


if __name__ == "__main__":
    main()
