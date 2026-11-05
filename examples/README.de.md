# DVB-T-Beispielempfänger für GNU Radio Companion

**[English version](./README.md)**

Dieses Verzeichnis enthält Beispiel-Flowgraphs für [GNU Radio Companion (GRC)](https://www.gnuradio.org/), die vollständige schmalbandige **DVB-T-Empfänger für digitales Amateurfunkfernsehen (DATV)** im 70-cm-Amateurfunkband implementieren. Sie kombinieren die vom Modul [gr-dl5eu](https://github.com/dl5eu/gr-dl5eu) bereitgestellten Empfangsblöcke mit den DVB-T-Blöcken von GNU Radio.

Als Empfangs-Frontend dient entweder ein **Analog Devices ADALM-PLUTO** oder ein **RTL-SDR-Stick**. Die vier Empfänger unterscheiden sich vor allem im Konzept zur Einstellung der SDR-Abtastrate und in den angebotenen Kanalbandbreiten. Alle vier erzeugen einen MPEG-Transportstrom (MPEG-TS), der über UDP ausgegeben und beispielsweise mit dem [VLC media player](https://www.videolan.org/vlc/) wiedergegeben werden kann.

> **Projektstatus:** Die Flowgraphs sind Beispiele und befinden sich weiterhin in der Entwicklung. Konfiguration, Implementierung und Empfangsverhalten können sich ohne Vorankündigung ändern.

## Verfügbare Flowgraphs

| Flowgraph | SDR-Frontend | SDR-Eingangsabtastrate | Wählbare DVB-T-Kanalbandbreiten |
| --- | --- | --- | --- |
| `dvbt_rx_pluto.grc` | ADALM-PLUTO | Fest **2,4 MS/s** | 2 MHz, 1 MHz, 500 kHz, 333 kHz, 250 kHz, 125 kHz, 66 kHz |
| `dvbt_rx_rtlsdr.grc` | RTL-SDR | Fest **2,4 MS/s** | 2 MHz, 1 MHz, 500 kHz, 333 kHz, 250 kHz, 125 kHz, 66 kHz |
| `dvbt_rx_rtlsdr_1_mhz.grc` | RTL-SDR | Abhängig von der gewählten Bandbreite | 1 MHz, 250 kHz |
| `dvbt_rx_rtlsdr_250_khz.grc` | RTL-SDR | Ca. **285,7 kS/s** (nur eine wählbare Bandbreite) | 250 kHz |

Die Tabelle nennt die **tatsächlich auswählbaren Einstellungen der vorliegenden GRC-Dateien**. Sie bedeutet nicht, dass mit jedem SDR ein stabiler Empfang garantiert ist. Die beiden Empfänger mit fester Abtastrate erzeugen die für schmale Kanäle nötigen Abtastraten durch Software-Resampling. Die beiden RTL-SDR-Varianten mit direkter Abtastrateneinstellung bieten nur Kanalbandbreiten an, deren Hardware-Abtastraten innerhalb der vom Standardtreiber `librtlsdr` unterstützten Bereiche liegen.

Alle vier Beispiele sind derzeit für den **DVB-T-Übertragungsmodus 2K** konfiguriert. Der von einigen `gr-dl5eu`-Blöcken ebenfalls unterstützte Modus 8K ist in den Flowgraphs ohne Änderungen nicht auswählbar.

## Funktionsweise der Empfänger

Jeder Flowgraph enthält die Verarbeitungsschritte, die zur Rückgewinnung von MPEG-TS-Paketen aus einem DVB-T-Signal erforderlich sind. Dazu gehören OFDM-Synchronisation und FFT, TPS-Decodierung, Demapping und Deinterleaving der Träger, Viterbi-Decodierung, Convolutional Deinterleaving, Reed-Solomon-Decodierung und Energy Descrambling.

Die Beispiele verwenden die eigenen Blöcke **DVB-T OFDM Synchronization**, **DVB-T TPS Decoder**, **DVB-T Reed-Solomon Decoder** und **DVB-T Energy Descramble** aus `gr-dl5eu`. Reed-Solomon-Decoder und Energy Descrambler arbeiten dabei zusammen, um bei MPEG-TS-Paketen mit nicht korrigierbaren Reed-Solomon-Fehlern das Transport-Error-Indicator-Bit zu setzen. Außerdem verwenden die Beispiel-Flowgraphs den **DVB-T Viterbi Decoder** aus `gr-dl5eu`. Der optionale **MPEG-TS Monitor** ist in diesen Beispielen nicht enthalten.

Die grafische Bedienoberfläche enthält Steuerelemente für die Übertragungsparameter sowie eine Spektrumanzeige, ein Konstellationsdiagramm und eine Anzeige des Frequenzfehlers. Diese erleichtern die Abstimmung und Fehlersuche. Die Schaltfläche **Restart** beendet den laufenden Flowgraph und startet das erzeugte Empfängerprogramm neu.

### Gemeinsame Empfangsparameter

| Einstellung | Auswahlmöglichkeiten bzw. Einstellbereich |
| --- | --- |
| Mittenfrequenz | GUI-Einstellbereich **430–440 MHz**; Anfangsvorgabe **437 MHz** |
| DVB-T-Kanalbandbreite | Abhängig vom Flowgraph; siehe Tabelle oben |
| Guardintervall | **1/4, 1/8, 1/16, 1/32** |
| Modulation | **QPSK, 16-QAM, 64-QAM** |
| Coderate | **1/2, 2/3, 3/4, 5/6, 7/8** |
| HF-Verstärkung | Einstellbar; Wertebereich abhängig vom SDR |
| Squelch | Einstellbar; **−200 dBFS** deaktiviert den Squelch des Synchronisators |
| Frequenzkorrektur | Bei den RTL-SDR-Varianten in **ppm** einstellbar |

Die eingestellten Übertragungsparameter müssen zum empfangenen Signal passen. Nicht jede Kombination aus Bandbreite, Guardintervall, Modulation und Coderate ist in der Praxis sinnvoll: Die Auswahl beeinflusst die mögliche Transportstrom-Datenrate und die erforderliche Signalqualität. Deshalb kann etwas Experimentieren nötig sein.

Anders als beim üblichen Fernsehempfang treten im Amateurfunk häufig Signale auf, verschwinden wieder oder werden kurzfristig durch Aussendungen anderer Stationen ersetzt. Der OFDM-Synchronisator von `gr-dl5eu` ist darauf ausgelegt, Signale zu erfassen und nach Unterbrechungen möglichst schnell wieder zu synchronisieren. Die Beispiele passen ihre eingestellten DVB-T-Parameter jedoch **nicht automatisch** an beliebige neue Sendesignale an.

## Beschreibung der einzelnen Flowgraphs

### `dvbt_rx_pluto.grc` — Empfänger mit ADALM-PLUTO

Dieser Flowgraph verwendet einen **ADALM-PLUTO** als Empfangs-Frontend. Aktiv ist die **IIO Pluto Source** von GNU Radio. Eine alternative Pluto-Quelle über SoapySDR ist in der GRC-Datei vorhanden, aber deaktiviert.

Der Pluto tastet unabhängig von der gewählten DVB-T-Kanalbandbreite kontinuierlich mit **2,4 MS/s** ab. Im jeweils ausgewählten Signalweg erzeugt ein **Rational Resampler** daraus die benötigte nominelle DVB-T-Abtastrate. Die Kanalbandbreite kann daher geändert werden, ohne die Hardware-Abtastrate des Pluto umzustellen.

Die größte auswählbare Kanalbandbreite beträgt **2 MHz**. Bei den Tests des Autors war damit der Empfang über ADALM-PLUTO erfolgreich. Das tatsächliche Ergebnis hängt jedoch von der Qualität des Sendesignals und dem verwendeten Rechner ab.

### `dvbt_rx_rtlsdr.grc` — RTL-SDR-Empfänger mit fester Abtastrate

Dieser Empfänger entspricht weitgehend der Pluto-Variante, verwendet jedoch einen **RTL-SDR-Stick**, der über eine **Soapy RTL-SDR Source** angesprochen wird. Die Hardware wird auf konstante **2,4 MS/s** eingestellt. Rational Resampler erzeugen die benötigten DVB-T-Verarbeitungsraten für die Kanalbandbreiten von **2 MHz** bis **66 kHz**.

Bei einem Wechsel der Kanalbandbreite bleibt die Hardware-Abtastrate unverändert. Allerdings kann die kontinuierliche Übertragung mit 2,4 MS/s manche RTL-SDR-Sticks und Rechner überfordern. Verlorene IQ-Samples können die OFDM-Synchronisation stören und Bildaussetzer verursachen. Insbesondere bedeutet die Auswahlmöglichkeit **2 MHz** nicht, dass ein RTL-SDR mit dieser Bandbreite zuverlässig DVB-T empfangen kann.

### `dvbt_rx_rtlsdr_1_mhz.grc` — RTL-SDR-Empfänger mit direkter Abtastrateneinstellung

Diese Variante ist für RTL-SDR-Hardware gedacht, die einen Datenstrom mit 2,4 MS/s nicht zuverlässig liefern kann. Anstatt mit einer festen Hardware-Abtastrate zu arbeiten und danach per Software umzurechnen, wird die **Abtastrate der RTL-SDR-Quelle direkt** entsprechend der gewählten DVB-T-Kanalbandbreite eingestellt.

Zur Verfügung stehen **1 MHz** und **250 kHz**. Dafür werden ungefähr **1,143 MS/s** beziehungsweise **285,7 kS/s** benötigt. Beide Werte liegen in den vom Standardtreiber `librtlsdr` unterstützten Bereichen. Die 2-MHz-Option wird bewusst nicht angeboten. Die Einträge für 500 kHz, 333 kHz, 125 kHz und 66 kHz sind auskommentiert, da die hierfür benötigten Hardware-Abtastraten außerhalb dieser Bereiche liegen.

Die gegenüber 2,4 MS/s reduzierte Abtastrate von 1,143 MS/s verringert die über USB übertragene Datenmenge und kann die Stabilität beim Einsatz eines RTL-SDR verbessern. Ein unterbrechungsfreier Empfang ist dadurch jedoch nicht garantiert.

### `dvbt_rx_rtlsdr_250_khz.grc` — RTL-SDR-Empfänger für 250-kHz-Kanäle

Diese Variante verwendet ebenfalls eine direkt eingestellte Hardware-Abtastrate, bietet jedoch **ausschließlich 250 kHz** DVB-T-Kanalbandbreite an. Entsprechend wird der RTL-SDR auf eine konstante Abtastrate von ungefähr **285,7 kS/s** eingestellt. Die anderen Bandbreiteneinträge sind auskommentiert und erscheinen nicht in der GUI.

Die niedrige Abtastrate reduziert die Datenübertragung über USB erheblich und liegt im unteren unterstützten Bereich des Standardtreibers. Der Flowgraph ist für Experimente mit besonders schmalbandigem DATV vorgesehen.

## Abtastratenkonzepte und Einschränkungen

Die nominelle Abtastrate für die hier verwendete DVB-T-Verarbeitung beträgt ungefähr das **8/7-Fache der gewählten Kanalbandbreite**. Die vier Empfänger unterscheiden sich darin, *wo* diese Abtastrate erzeugt wird:

- **Varianten mit fester Abtastrate:** Das SDR tastet mit **2,4 MS/s** ab. Die niedrigeren Verarbeitungsraten entstehen durch **Rational Resampler** in GNU Radio.
- **Varianten mit direkter Abtastrateneinstellung:** Die RTL-SDR-Quelle wird fest auf die zur gewählten Bandbreite passende Abtastrate eingestellt. Ein zusätzlicher Rational Resampler ist in diesen Varianten nicht notwendig.

### Abtastratengrenzen des RTL-SDR

Die [Schnittstellenbeschreibung von `librtlsdr`](https://github.com/osmocom/rtl-sdr/blob/master/include/rtl-sdr.h) nennt zwei zulässige Hardware-Abtastratenbereiche: **225.001–300.000 Samples/s** und **900.001–3.200.000 Samples/s**. Außerdem warnt die Dokumentation vor möglichen Sampleverlusten bei Raten über 2,4 MS/s. Auch bei zulässigen Werten hängt ein stabiler Empfang von Hardware und Rechner ab.

Die folgenden Werte verdeutlichen, warum bei den **RTL-SDR-Varianten mit direkter Abtastrateneinstellung** nur bestimmte Bandbreiten freigeschaltet sind:

| DVB-T-Kanalbandbreite | Nominelle direkte SDR-Abtastrate | Standard-`librtlsdr` |
| --- | ---: | --- |
| 1 MHz | 1.142,9 kS/s | Im gültigen Bereich; in der 1-MHz-Variante angeboten |
| 500 kHz | 571,4 kS/s | **Außerhalb des Bereichs; deaktiviert** |
| 333 kHz | 380,6 kS/s | **Außerhalb des Bereichs; deaktiviert** |
| 250 kHz | 285,7 kS/s | Im gültigen Bereich; in beiden direkt abtastenden Varianten angeboten |
| 125 kHz | 142,9 kS/s | **Außerhalb des Bereichs; deaktiviert** |
| 66 kHz | 75,4 kS/s | **Außerhalb des Bereichs; deaktiviert** |

Die **2-MHz-Option** ist in den direkt abtastenden Varianten ebenfalls bewusst ausgeschlossen, obwohl die zugehörige nominelle Abtastrate von 2,286 MS/s grundsätzlich im dokumentierten Treiberbereich liegt. Die oben genannten nicht unterstützten Bandbreiteneinträge sind nur noch als **auskommentierte Zeilen** im eingebetteten Python-Modul der GRC-Dateien vorhanden. Sie sind **keine auswählbaren GUI-Optionen**. Die Varianten mit fester Abtastrate können die niedrigeren Verarbeitungsraten durch Software-Resampling erzeugen, sofern das SDR einen ausreichend zuverlässigen Datenstrom mit 2,4 MS/s liefert.

## Ausgabe des MPEG-Transportstroms über UDP

Alle vier Flowgraphs verwenden die gleichen Einstellungen der GNU-Radio-**UDP Sink**:

| Eigenschaft | Wert |
| --- | --- |
| Zieladresse | `230.0.0.1` (IPv4-Multicast) |
| UDP-Port | `2000` |
| Dateninhalt | MPEG-Transportstrom mit 188-Byte-TS-Paketen |
| UDP-Nutzdatenlänge | `1316` Byte (7 × 188 Byte) |

Die IP-Adresse lautet `udp://230.0.0.1:2000`. Zum **Empfang** des Multicast-Streams mit VLC wählt man *Medien → Netzwerkstream öffnen* und gibt Folgendes ein:

```text
udp://@230.0.0.1:2000
```

Das `@` in der VLC-Empfangsadresse ist beabsichtigt. Die Wiedergabe kann auf dem Empfangsrechner oder einem anderen Gerät erfolgen, das die an diese Multicast-Gruppe gerichteten Pakete empfängt. Falls nötig, müssen Firewallregeln, Netzwerkschnittstellen und Multicast-Routing passend konfiguriert werden.

## Inbetriebnahme und Speichern der Einstellungen

1. GNU Radio einschließlich GNU Radio Companion, das OOT-Modul **gr-dl5eu** und die zum gewählten Flowgraph gehörende SDR-Unterstützung installieren: **IIO** für die aktive Pluto-Quelle beziehungsweise **SoapySDR / Soapy RTL-SDR** für die RTL-SDR-Varianten.
2. Die gewünschte `.grc`-Datei in GNU Radio Companion öffnen, das Empfängerprogramm erzeugen und mit angeschlossenem SDR starten.
3. Mittenfrequenz, Bandbreite, Guardintervall, Modulation und Coderate passend zur Sendestation einstellen. Bei Bedarf HF-Verstärkung, Squelch des Synchronisators und RTL-SDR-Frequenzkorrektur anpassen.
4. `udp://@230.0.0.1:2000` in VLC oder einem anderen MPEG-TS-fähigen Player öffnen. Die Schaltfläche **Restart** verwenden, falls nach Änderungen ein vollständiger Neustart des Empfängers erforderlich ist.

Die Empfänger nutzen die GNU-Radio-Companion-Blöcke **Variable Config**, um GUI-Einstellungen in INI-Dateien zu speichern und beim nächsten Start wiederherzustellen. **Jeder Flowgraph besitzt jetzt eine eigene Konfigurationsdatei:**

| Flowgraph | Konfigurationsdatei |
| --- | --- |
| `dvbt_rx_pluto.grc` | `config_pluto.ini` |
| `dvbt_rx_rtlsdr.grc` | `config_rtlsdr.ini` |
| `dvbt_rx_rtlsdr_1_mhz.grc` | `config_rtlsdr_1_mhz.ini` |
| `dvbt_rx_rtlsdr_250_khz.grc` | `config_rtlsdr_250_khz.ini` |

Die Dateinamen sind relativ angegeben. Die Dateien werden daher bezogen auf das Arbeitsverzeichnis des gestarteten Empfängerprogramms gelesen oder angelegt. Die getrennten Dateinamen verhindern, dass sich die unterschiedlichen Bandbreitenauswahlen der RTL-SDR-Varianten beim Betrieb aus demselben Verzeichnis gegenseitig beeinflussen.

## Praktische Hinweise

Ein kostengünstiger RTL-SDR kann für schmalbandige DATV-Experimente eingesetzt werden. Die Zuverlässigkeit hängt jedoch vom einzelnen Stick, USB-Anschluss, Rechner, Sendesignal und den gewählten Parametern ab. Bei den Tests des Autors lag **der DVB-T-Empfang mit 1 MHz Bandbreite bei den verwendeten RTL-SDR-Sticks bereits nahe an deren praktischer Leistungsgrenze**; gelegentliche Unterbrechungen und Neusynchronisationen traten auf. Mit einem **ADALM-PLUTO** waren die Ergebnisse insgesamt besser, und im getesteten Aufbau war auch **2 MHz** Bandbreite möglich. Diese Erfahrungen sind keine allgemeine Garantie für andere Geräte.

Eine geringe Kanalbandbreite allein gewährleistet noch keinen funktionierenden Videoempfang. Der Sender muss ein dazu passendes DVB-T-Signal erzeugen, und die MPEG-TS-Datenrate muss innerhalb der verfügbaren Übertragungskapazität liegen. Guardintervall, Modulation und Coderate beeinflussen sowohl die nutzbare Datenrate als auch die Anforderungen an die Empfangsbedingungen.

## Rückmeldungen und Weiterentwicklung

Die Beispiele sind als Ausgangspunkt für weitere Experimente gedacht. Rückmeldungen anderer Funkamateure mit verschiedenen SDRs, DATV-Sendern, Bandbreiten und Empfangsbedingungen sind willkommen. Besonders interessant sind Erfahrungen mit dem Synchronisationsverhalten, Sampleverlusten und der Stabilität der Videowiedergabe.

**Autor:** Ralf Gorholt (DL5EU)  
**E-Mail:** [dl5eu@darc.de](mailto:dl5eu@darc.de)

Die Beispiele gehören zum Projekt [gr-dl5eu](https://github.com/dl5eu/gr-dl5eu). Die Lizenzbedingungen stehen in der Datei [LICENSE](https://github.com/dl5eu/gr-dl5eu/blob/main/LICENSE) im Repository.
