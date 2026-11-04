# gr-dl5eu — Narrowband DVB-T Reception with GNU Radio

**gr-dl5eu** is an out-of-tree (OOT) module for [GNU Radio](https://www.gnuradio.org/) that provides additional signal-processing blocks for receiving narrowband DVB-T signals. Its primary goal is to make digital amateur television (DATV) reception in the 70 cm amateur radio band possible with affordable software-defined radio (SDR) hardware, including RTL-SDR dongles.

The module also includes example receiver flowgraphs for GNU Radio Companion (GRC). The decoded MPEG transport stream can be sent over UDP and played using [VLC media player](https://www.videolan.org/vlc/) or another suitable player.

> **Project status:** This project is a work in progress. Its blocks, example flowgraphs, and configuration options are subject to change.

## Background and Motivation

The starting point for this project was the search for a simple, inexpensive way to receive DVB-T transmissions with relatively small channel bandwidths in the 70 cm amateur radio band. IARU Region 1 has recommended a **maximum bandwidth of 2 MHz for digital amateur television (DATV)** in this band. This is a technical recommendation, not a substitute for the applicable national regulations and frequency plans. See the [IARU Region 1 VHF Managers Handbook, section 8.8.11](https://www.darc.de/fileadmin/filemounts/referate/vus/iaru/VHF_Handbook_V7_51.pdf).

Many radio amateurs interested in SDR already own an RTL-SDR dongle. It therefore seemed worthwhile to investigate whether such a low-cost receiver, together with GNU Radio, could be used to receive narrowband DVB-T signals.

GNU Radio already provides most of the blocks needed to construct a DVB-T receiver through its [`gr-dtv` module](https://github.com/gnuradio/gnuradio/tree/main/gr-dtv). Initial experiments showed that a complete receiver could indeed be assembled and its decoded transport stream delivered to a UDP port. However, although reception was possible, it was not sufficiently reliable for continuous video playback.

## Development History

### OFDM Synchronization

A major difficulty was reliable OFDM symbol acquisition. Small differences between the transmitter and receiver sampling clocks cause the position of the cyclic prefix (guard interval) to drift relative to the receiver's sampling window. Without adequate sampling-rate correction and timing tracking, the symbol acquisition process can eventually lose synchronization. Reacquisition then interrupts the decoded video stream.

An important starting point for improving this behaviour was the [`gr-isdbt`](https://github.com/git-artes/gr-isdbt) project developed by Federico "Larroca" La Rocca and collaborators. Its **OFDM Synchronization** block combines OFDM symbol acquisition, FFT processing, and synchronization corrections, including compensation for sampling-clock errors. The project also provides a **TMCC Decoder** for the ISDB-T transmission parameters.

Building on these implementations, I developed corresponding blocks for DVB-T. The OFDM synchronization and transmission-parameter decoding had to be adapted to the different signal structures of DVB-T and ISDB-T, including their pilot carriers and signaling mechanisms. DVB-T uses Transmission Parameter Signalling (TPS) rather than ISDB-T's TMCC.

### Reed-Solomon Decoding and Energy Descrambling

Further receiver tests exposed another problem near the end of the decoding chain. GNU Radio's standard DVB-T Reed-Solomon decoder can detect codewords that cannot be corrected, but the affected MPEG transport packets are not explicitly marked as erroneous for downstream processing. Consequently, corrupted packet contents may pass through the standard energy descrambler without the MPEG-TS error indication being set. This can contribute to difficult-to-explain playback disturbances or frozen video.

To improve error handling, **gr-dl5eu** provides a modified pair of blocks:

1. **DVB-T Reed-Solomon Decoder** corrects errors where possible and attaches an `rs_uncorrectable` stream tag to identify any codeword that could not be corrected.
2. **DVB-T Energy Descramble** performs the usual energy descrambling and uses this information to set the MPEG-TS `transport_error_indicator` (TEI) bit in the corresponding 188-byte transport packet.

The TEI flag identifies an erroneous packet; it does not repair its contents. Other reception problems can still cause interruptions, but this change prevents uncorrectable Reed-Solomon errors from passing through this part of the receiver without an appropriate transport-stream error indication.

**For receivers built with gr-dl5eu, the DVB-T Reed-Solomon Decoder and DVB-T Energy Descramble blocks from this module are strongly recommended as a pair instead of their standard GNU Radio counterparts.**

## Blocks Provided by gr-dl5eu

### Main Receiver Blocks

| Block | Function |
| --- | --- |
| **DVB-T OFDM Synchronization** | Acquires and tracks DVB-T OFDM symbols, performs FFT processing, and compensates for frequency and sampling-clock deviations. |
| **DVB-T TPS Decoder** | Decodes DVB-T Transmission Parameter Signalling and extracts payload carriers for subsequent processing. |
| **DVB-T Reed-Solomon Decoder** | Performs outer forward-error-correction decoding and marks uncorrectable codewords using stream tags. |
| **DVB-T Energy Descramble** | Reverses DVB-T energy dispersal and sets the transport error indicator for packets marked as uncorrectable. |

The OFDM Synchronization and TPS Decoder blocks support the DVB-T **2K and 8K transmission modes**.

### Additional Diagnostic Blocks

Two additional blocks were created primarily for development, troubleshooting, and analysis:

- **DVB-T Viterbi Decoder** — an alternative Viterbi decoder useful during investigation of the decoding chain. A custom Viterbi decoder is not generally required for a conventional DVB-T receiver, although the current example flowgraphs use the version supplied with this module.
- **MPEG-TS Monitor** — a transparent monitor for 188-byte MPEG transport packets. It can report synchronization, transport-error, and continuity-counter problems and can record diagnostic statistics to a CSV log. It is optional for normal reception.

## Example Receiver Flowgraphs

The [`examples`](https://github.com/dl5eu/gr-dl5eu/tree/main/examples) directory contains two GNU Radio Companion flowgraphs that demonstrate the use of the module:

| Flowgraph | Receiver hardware |
| --- | --- |
| [`dvbt_rx_rtlsdr.grc`](https://github.com/dl5eu/gr-dl5eu/blob/main/examples/dvbt_rx_rtlsdr.grc) | RTL-SDR dongle |
| [`dvbt_rx_pluto.grc`](https://github.com/dl5eu/gr-dl5eu/blob/main/examples/dvbt_rx_pluto.grc) | ADALM-PLUTO SDR |

Both examples provide controls for adapting reception to the transmitted signal, including **channel bandwidth, guard interval, modulation, and code rate**, as well as receiver-specific settings such as center frequency and gain.

The flowgraphs use GNU Radio's **Variable Config** blocks to store selected settings in configuration files (`config_rtlsdr.ini` and `config_pluto.ini`, respectively), so that these settings can be reused when the receiver is started again.

The decoded MPEG transport stream is sent to a UDP sink and can be viewed with VLC or another player capable of receiving an MPEG-TS stream over UDP.

The receiver settings must match the parameters of the transmitted DVB-T signal.

## Practical Limitations and Test Results

### RTL-SDR

Although RTL-SDR devices can nominally deliver sample rates above 2 MS/s, loss-free reception cannot be assumed in every hardware and software configuration. Missing samples may occur without a corresponding warning from the driver. Even occasional sample losses can disturb OFDM synchronization and cause visible interruptions in a digital television stream.

In my tests, the RTL-SDR dongles available to me were **just about capable of receiving DVB-T signals with a channel bandwidth of 1 MHz**. Short interruptions and occasional resynchronization still occurred. These results should not be generalized to all RTL-SDR models or individual devices: performance can vary with the dongle, USB connection, computer, driver, and operating conditions.

For this application, an RTL-SDR is operating close to its practical limits. Reliable continuous reception cannot be guaranteed.

### Influence of Transmitter Signal Quality

Receiver performance also depends strongly on the quality of the transmitted DVB-T signal. With a transmitter based on a **HiDes chipset**, reception using an RTL-SDR worked reasonably well under the test conditions.

By comparison, a DVB-T signal generated using a **Raspberry Pi 4B**, **Portsdown software**, and an **ADALM-PLUTO** produced more frequent interruptions at the receiver, despite using the same DVB-T transmission parameters. This observation describes the particular tested setups; it should not be taken as a general assessment of either transmitter platform.

### ADALM-PLUTO as a Receiver

Tests using an **ADALM-PLUTO** as the receiving SDR were more successful. In this configuration, reception was also possible with a **2 MHz DVB-T channel bandwidth**. An SDR with more dependable sample delivery therefore offers a useful alternative when an RTL-SDR is not sufficiently reliable.

## Development Environment

The module is built as a GNU Radio 3.10 OOT module using C++, Python bindings, and GNU Radio Companion block definitions. Development and testing have primarily used **GNU Radio 3.10.12 under Windows 11 with RadioConda**.

The repository contains a CMake build system and a [Conda build recipe](https://github.com/dl5eu/gr-dl5eu/tree/main/.conda/recipe). Other environments and hardware combinations may require additional testing.

## Feedback and Contributions

**gr-dl5eu is under active development**, and the software may continue to change as testing reveals further issues or opportunities for improvement.

I would particularly welcome reports from other radio amateurs who try receiving narrowband DVB-T signals using this module and its example flowgraphs. Reports about the SDR hardware used, transmission parameters, reception stability, and any observed problems would be especially valuable.

## Acknowledgements

The OFDM synchronization and TPS decoding work was inspired by and developed from ideas and implementations in [`gr-isdbt`](https://github.com/git-artes/gr-isdbt) by **Federico "Larroca" La Rocca and collaborators**. Thanks are also due to the developers of [GNU Radio](https://github.com/gnuradio/gnuradio), whose existing DVB-T blocks provide the foundation of the receiver.

## Author

**Ralf Gorholt (DL5EU)**  
Email: [dl5eu@darc.de](mailto:dl5eu@darc.de)

## License

This project is distributed under the **GNU General Public License, version 3 or later (GPL-3.0-or-later)**. See the repository's [LICENSE](https://github.com/dl5eu/gr-dl5eu/blob/main/LICENSE) file for details.
