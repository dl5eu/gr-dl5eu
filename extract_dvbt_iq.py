#!/usr/bin/env python3
"""Extract a small raw GNU Radio complex64 IQ window around a CP_LOST event."""

import argparse
from pathlib import Path

BYTES_PER_SAMPLE = 8       # GNU Radio complex: float32 real + float32 imaginary
SAMPLES_BEFORE = 100_000   # about 87.5 ms at 1,142,857 samples/s
SAMPLES_AFTER = 100_000
COPY_CHUNK_BYTES = 1024 * 1024


def main():
    parser = argparse.ArgumentParser(
        description="IQ-Ausschnitt um einen CP_LOST-Eintrag extrahieren."
    )
    parser.add_argument("iq_file", nargs="?", help="IQ-Datei des File Sink (complex)")
    parser.add_argument("window_start", nargs="?", type=int,
                        help="Sampleposition aus CP_LOST: window_start=...")
    args = parser.parse_args()

    path_text = args.iq_file
    if path_text is None:
        path_text = input("Pfad zur IQ-Aufzeichnung: ").strip().strip('"')
    source = Path(path_text)
    if not source.is_file():
        parser.error(f"Die IQ-Datei existiert nicht: {source}")

    event = args.window_start
    if event is None:
        try:
            event = int(input("window_start aus der zugehoerigen CP_LOST-Meldung: "))
        except ValueError:
            parser.error("window_start muss eine ganze Zahl sein")
    if event < 0:
        parser.error("window_start darf nicht negativ sein")

    file_bytes = source.stat().st_size
    if file_bytes % BYTES_PER_SAMPLE:
        parser.error("Dateigroesse ist kein Vielfaches von 8 Byte."
                     " Ist der File-Sink-Typ wirklich Complex?")
    total_samples = file_bytes // BYTES_PER_SAMPLE
    if event >= total_samples:
        parser.error(f"window_start={event} liegt ausserhalb der Datei "
                     f"(0 bis {total_samples - 1}).")

    first = max(0, event - SAMPLES_BEFORE)
    end = min(total_samples, event + SAMPLES_AFTER)  # exclusive
    count = end - first

    output = source.with_name(f"{source.stem}_cp_{event}.cfile")
    metadata = output.with_suffix(".txt")
    if output.exists() or metadata.exists():
        parser.error("Ausgabedatei existiert bereits; keine Datei wurde ueberschrieben: "
                     f"{output}")

    remaining = count * BYTES_PER_SAMPLE
    with source.open("rb") as src, output.open("wb") as dst:
        src.seek(first * BYTES_PER_SAMPLE)
        while remaining:
            data = src.read(min(COPY_CHUNK_BYTES, remaining))
            if not data:
                raise IOError("Unerwartetes Dateiende beim Lesen")
            dst.write(data)
            remaining -= len(data)

    metadata.write_text(
        f"Quelldatei: {source}\n"
        f"Original-Samplezahl: {total_samples}\n"
        f"CP_LOST window_start: {event}\n"
        f"Erstes Sample im Ausschnitt (original): {first}\n"
        f"Letztes Sample im Ausschnitt (original, inklusive): {end - 1}\n"
        f"Anzahl komplexer Samples: {count}\n"
        f"Format: GNU Radio Complex, float32 I und float32 Q, little-endian\n"
        f"Abtastrate: 1142857.021756 Samples/s\n",
        encoding="utf-8",
    )

    print(f"IQ-Ausschnitt: {output}")
    print(f"Zuordnung der Samplepositionen: {metadata}")
    print(f"Original-Samples: {first} bis {end - 1} ({count} Samples)")
    print(f"Groesse: {count * BYTES_PER_SAMPLE:,} Bytes")


if __name__ == "__main__":
    main()
