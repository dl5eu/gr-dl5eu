# Example DVB-T Receivers for GNU Radio Companion

**[Deutsche Fassung](./README.de.md)**

This directory contains example flowgraphs for [GNU Radio Companion (GRC)](https://www.gnuradio.org/) implementing complete narrowband **DVB-T receivers for digital amateur television (DATV)** in the 70 cm amateur radio band. They combine the receiver blocks provided by [gr-dl5eu](https://github.com/dl5eu/gr-dl5eu) with DVB-T processing blocks included in GNU Radio.

The four receivers use either an **Analog Devices ADALM-PLUTO** or an **RTL-SDR dongle** as the radio front end. Their main differences are the SDR sampling-rate strategy and the channel bandwidths offered. All four produce an MPEG transport stream (MPEG-TS) and send it over UDP for playback with [VLC media player](https://www.videolan.org/vlc/) or another suitable application.

> **Project status:** These flowgraphs are examples and remain a work in progress. Their configuration, implementation and reception performance may change without notice as development continues.

## Available Flowgraphs

| Flowgraph | SDR front end | SDR input sampling rate | Selectable DVB-T channel bandwidths |
| --- | --- | --- | --- |
| `dvbt_rx_pluto.grc` | ADALM-PLUTO | Fixed at **2.4 MS/s** | 2 MHz, 1 MHz, 500 kHz, 333 kHz, 250 kHz, 125 kHz, 66 kHz |
| `dvbt_rx_rtlsdr.grc` | RTL-SDR | Fixed at **2.4 MS/s** | 2 MHz, 1 MHz, 500 kHz, 333 kHz, 250 kHz, 125 kHz, 66 kHz |
| `dvbt_rx_rtlsdr_1_mhz.grc` | RTL-SDR | Set according to the selected bandwidth | 1 MHz, 250 kHz |
| `dvbt_rx_rtlsdr_250_khz.grc` | RTL-SDR | Approximately **285.7 kS/s** (one selectable bandwidth) | 250 kHz |

These are the **currently selectable options in the supplied GRC files**, not a guarantee of reliable reception with every SDR. The two fixed-rate receivers use software resampling for narrow channels. The two direct-rate RTL-SDR variants offer only bandwidths for which their requested hardware sampling rates fall within the standard `librtlsdr` ranges.

All four examples are configured for **DVB-T 2K mode**. The 8K mode supported by some `gr-dl5eu` blocks is not selectable in these flowgraphs without modifications.

## Receiver Operation

Each flowgraph implements the processing chain needed to recover MPEG-TS packets from a DVB-T signal. The main stages include OFDM synchronization and FFT processing, TPS decoding, carrier demapping and deinterleaving, Viterbi decoding, convolutional deinterleaving, Reed-Solomon decoding and energy descrambling.

The examples use the custom **DVB-T OFDM Synchronization**, **DVB-T TPS Decoder**, **DVB-T Reed-Solomon Decoder** and **DVB-T Energy Descramble** blocks from `gr-dl5eu`. In particular, the Reed-Solomon decoder and energy descrambler cooperate to set the transport error indicator on MPEG-TS packets whose Reed-Solomon errors could not be corrected. The example flowgraphs also use the `gr-dl5eu` **DVB-T Viterbi Decoder**. The optional **MPEG-TS Monitor** is not included in these examples.

The graphical interface provides transmission-parameter controls, a spectrum display, a constellation display and a frequency-error indication for tuning and troubleshooting. The **Restart** button stops the running flowgraph and restarts the generated receiver program.

The **Resynchronize** button requests a fresh OFDM acquisition from the synchronizer associated with the currently selected guard interval, without stopping or restarting the receiver program. It resets the synchronizer's timing and frequency tracking and signals downstream processing blocks to reacquire synchronization. This can be useful after signal loss or if reception does not recover automatically. Unlike **Restart**, it keeps the flowgraph and SDR front end running and does not change the selected receiver parameters.

### Common Receiver Settings

| Setting | Available values or range |
| --- | --- |
| Center frequency | GUI tuning range **430–440 MHz**; initial default **437 MHz** |
| DVB-T channel bandwidth | Depends on the flowgraph; see the table above |
| Guard interval | **1/4, 1/8, 1/16, 1/32** |
| Modulation | **QPSK, 16-QAM, 64-QAM** |
| Code rate | **1/2, 2/3, 3/4, 5/6, 7/8** |
| RF gain | Adjustable; the range depends on the selected SDR |
| Squelch | Adjustable; **−200 dBFS** disables the synchronizer's squelch |
| Frequency correction | Adjustable in **ppm** on the RTL-SDR variants |

The transmission parameters must match the signal being received. Not every combination of bandwidth, guard interval, modulation and code rate is useful in practice: the choice affects the available transport-stream data rate and required signal quality. Experimentation may be necessary.

Unlike conventional broadcast television reception, amateur DATV reception frequently involves signals that appear, disappear or are replaced by transmissions from other stations. The `gr-dl5eu` OFDM synchronizer is designed to acquire a signal and regain synchronization promptly after interruptions. However, the examples do **not** automatically adapt their DVB-T parameter settings to an arbitrary new transmission.

## Individual Receiver Flowgraphs

### `dvbt_rx_pluto.grc` — ADALM-PLUTO Receiver

This flowgraph uses an **ADALM-PLUTO** as the receiver front end. Its active source is GNU Radio's **IIO Pluto source**; an alternative SoapySDR Pluto source is present in the GRC file but disabled.

The Pluto samples continuously at **2.4 MS/s**, regardless of the selected DVB-T channel bandwidth. The selected processing branch uses a **Rational Resampler** to derive the required nominal DVB-T sampling rate from this fixed rate. Changing bandwidth therefore does not require changing the Pluto's hardware sample rate.

The widest selectable channel is **2 MHz**. Reception at this bandwidth was successful in the author's tests with an ADALM-PLUTO, but actual performance depends on signal quality and the host system.

### `dvbt_rx_rtlsdr.grc` — RTL-SDR Receiver with Fixed Sampling Rate

This receiver is similar to the Pluto version, but uses an **RTL-SDR dongle** through a **Soapy RTL-SDR Source**. It requests a constant hardware sampling rate of **2.4 MS/s**, and Rational Resampler blocks derive the required DVB-T processing rates for bandwidths from **2 MHz** down to **66 kHz**.

The hardware sampling rate remains unchanged when the channel bandwidth changes. However, sustained reception at 2.4 MS/s may be demanding for some RTL-SDR dongles and host systems. Lost IQ samples can disrupt OFDM synchronization and cause video interruptions. In particular, the presence of a **2 MHz** bandwidth option does not guarantee reliable 2 MHz reception with an RTL-SDR.

### `dvbt_rx_rtlsdr_1_mhz.grc` — RTL-SDR Receiver with Direct Sampling-Rate Selection

This variant is intended for RTL-SDR hardware that cannot reliably maintain a 2.4 MS/s stream. Instead of sampling at a fixed rate and resampling in software, it configures the **RTL-SDR source sampling rate directly** according to the selected DVB-T channel bandwidth.

The available bandwidths are **1 MHz** and **250 kHz**, requiring approximately **1.143 MS/s** and **285.7 kS/s**, respectively. Both requested rates fall within the standard `librtlsdr` ranges. The 2 MHz option is intentionally not offered; the 500 kHz, 333 kHz, 125 kHz and 66 kHz entries are commented out because their corresponding direct hardware sampling rates are outside these ranges.

Using 1.143 MS/s instead of 2.4 MS/s may improve stability with a bandwidth-limited RTL-SDR setup. This does not guarantee uninterrupted reception.

### `dvbt_rx_rtlsdr_250_khz.grc` — RTL-SDR Receiver for 250 kHz Channels

This version uses the same direct hardware sampling-rate approach but offers **only 250 kHz** DVB-T bandwidth. Consequently, it requests a constant nominal RTL-SDR sampling rate of approximately **285.7 kS/s**. The other bandwidth entries are commented out and do not appear in the GUI.

The low sample rate lies within the standard driver's supported lower range. This variant is intended for experimentation with very narrow DATV channels.

## Sampling-Rate Concepts and Limitations

For the DVB-T processing chain used here, the nominal complex sampling rate is approximately **8/7 of the selected channel bandwidth**. The difference between the four receivers is *where* that rate is produced:

- **Fixed-rate variants:** The SDR samples at **2.4 MS/s**. GNU Radio **Rational Resampler** blocks generate the lower processing rates.
- **Direct-rate variants:** The RTL-SDR source itself is configured to produce the rate required for the selected bandwidth. No separate rational resampler is needed in these variants.

### RTL-SDR Sampling-Rate Limits

The standard [`librtlsdr` driver API](https://github.com/osmocom/rtl-sdr/blob/master/include/rtl-sdr.h) documents two allowed hardware sampling-rate ranges: **225,001–300,000 samples/s** and **900,001–3,200,000 samples/s**. It also warns that sample loss is to be expected at rates above 2.4 MS/s. Actual reception at an allowed rate still depends on hardware and host performance.

The following values illustrate why only particular bandwidths are enabled in the **direct-rate RTL-SDR** examples:

| DVB-T channel bandwidth | Nominal direct SDR sampling rate | Standard `librtlsdr` range |
| --- | ---: | --- |
| 1 MHz | 1,142.9 kS/s | Supported range; offered by the 1 MHz variant |
| 500 kHz | 571.4 kS/s | **Outside range; disabled** |
| 333 kHz | 380.6 kS/s | **Outside range; disabled** |
| 250 kHz | 285.7 kS/s | Supported range; offered by both direct-rate variants |
| 125 kHz | 142.9 kS/s | **Outside range; disabled** |
| 66 kHz | 75.4 kS/s | **Outside range; disabled** |

The **2 MHz** option is also excluded from the direct-rate variants by design, although its nominal 2.286 MS/s sampling rate falls within the driver's documented range. The unsupported entries above remain only as **commented-out source lines** inside the GRC embedded Python module; they are **not selectable GUI options**. The fixed-rate variants can generate the lower processing rates through software resampling, provided the SDR supplies a sufficiently reliable 2.4 MS/s stream.

## UDP Transport-Stream Output

All four flowgraphs use the same GNU Radio **UDP Sink** settings:

| Property | Value |
| --- | --- |
| Destination address | `230.0.0.1` (IPv4 multicast) |
| UDP port | `2000` |
| Data | MPEG transport stream containing 188-byte TS packets |
| UDP payload size | `1316` bytes (7 × 188 bytes) |

The transmission destination is `udp://230.0.0.1:2000`. To **receive** the multicast stream in VLC, choose *Media → Open Network Stream* and enter:

```text
udp://@230.0.0.1:2000
```

The `@` in VLC's receiving URL is intentional. Playback may take place on the receiver computer or another device that can receive packets addressed to this multicast group. Firewall rules, network interfaces and multicast routing may need to be configured appropriately.

## Running the Examples and Saving Settings

1. Install GNU Radio with GNU Radio Companion, the **gr-dl5eu** OOT module and the SDR source support appropriate to the flowgraph: **IIO** for the active Pluto source, or **SoapySDR / Soapy RTL-SDR** for the RTL-SDR variants.
2. Open the desired `.grc` file in GNU Radio Companion, generate and run the receiver with the SDR connected.
3. Set the center frequency, bandwidth, guard interval, modulation and code rate to match the transmitting station. Adjust RF gain, synchronizer squelch and, if needed, RTL-SDR frequency correction.
4. Open `udp://@230.0.0.1:2000` in VLC or another MPEG-TS-capable player. Use the **Restart** button if a full receiver restart is necessary after changing the configuration.

The receivers use GNU Radio Companion **Variable Config** blocks to store GUI settings in INI files and restore them in later runs. **Each flowgraph now has its own configuration file:**

| Flowgraph | Configuration file |
| --- | --- |
| `dvbt_rx_pluto.grc` | `config_pluto.ini` |
| `dvbt_rx_rtlsdr.grc` | `config_rtlsdr.ini` |
| `dvbt_rx_rtlsdr_1_mhz.grc` | `config_rtlsdr_1_mhz.ini` |
| `dvbt_rx_rtlsdr_250_khz.grc` | `config_rtlsdr_250_khz.ini` |

These are relative filenames, so the files are created or read relative to the receiver program's working directory. Separate filenames prevent the different RTL-SDR bandwidth selections from interfering with one another when the examples are run from the same directory.

## Practical Notes

A low-cost RTL-SDR can be useful for narrowband DATV experiments. Nevertheless, reliable reception depends on the individual dongle, USB connection, host computer, transmitted signal and selected parameters. In the author's tests, **1 MHz DVB-T reception was already close to the practical limits of the RTL-SDR dongles used**, with occasional interruptions and resynchronization. Reception with an **ADALM-PLUTO** was generally more successful; **2 MHz** reception was also possible in the tested setup. These are experimental observations, not performance guarantees for other equipment.

A narrow channel alone does not ensure usable video. The transmitter must generate a matching DVB-T signal, and the MPEG-TS bit rate must fit the available channel capacity. Guard interval, modulation and code rate affect both the usable data rate and the reception conditions required.

## Feedback and Development

These examples are intended as a starting point for further experimentation. Reports from other radio amateurs using different SDRs, DATV transmitters, bandwidths and reception conditions are welcome, especially regarding synchronization behavior, lost samples and video playback stability.

**Author:** Ralf Gorholt (DL5EU)  
**Email:** [dl5eu@darc.de](mailto:dl5eu@darc.de)

The examples are part of the [gr-dl5eu project](https://github.com/dl5eu/gr-dl5eu). For licensing terms, see the repository's [LICENSE](https://github.com/dl5eu/gr-dl5eu/blob/main/LICENSE).
