## [English version](../README.md)

# gr-dl5eu — Schmalbandiger DVB-T-Empfang mit GNU Radio

**gr-dl5eu** ist ein Out-of-Tree-Modul (OOT) für [GNU Radio](https://www.gnuradio.org/), das zusätzliche Signalverarbeitungsblöcke für den Empfang schmalbandiger DVB-T-Signale bereitstellt. Das Hauptziel besteht darin, den Empfang von digitalem Amateurfunkfernsehen (DATV) im 70-cm-Amateurfunkband mit kostengünstiger SDR-Hardware (Software Defined Radio), darunter auch RTL-SDR-Sticks, zu ermöglichen.

Das Modul enthält außerdem Beispiel-Empfänger-Flowgraphs für GNU Radio Companion (GRC). Der decodierte MPEG-Transportstrom kann über UDP ausgegeben und mit dem [VLC Media Player](https://www.videolan.org/vlc/) oder einem anderen geeigneten Wiedergabeprogramm abgespielt werden.

> **Projektstatus:** Dieses Projekt befindet sich noch in der Entwicklung (*work in progress*). Die Blöcke, Beispiel-Flowgraphs und Konfigurationsmöglichkeiten können sich jederzeit ohne Ankündigung ändern.

## Hintergrund und Motivation

Ausgangspunkt dieses Projekts war die Suche nach einer einfachen und kostengünstigen Möglichkeit, DVB-T-Aussendungen mit geringer Kanalbandbreite (max. 2 MHz) im 70-cm-Amateurfunkband zu empfangen.

Viele Funkamateure, die sich mit SDR beschäftigen, besitzen bereits einen RTL-SDR-Stick. Deshalb lag es nahe zu untersuchen, ob sich ein solcher preisgünstiger Empfänger zusammen mit GNU Radio für den Empfang schmalbandiger DVB-T-Signale verwenden lässt.

GNU Radio stellt mit seinem Modul [`gr-dtv`](https://github.com/gnuradio/gnuradio/tree/main/gr-dtv) bereits die Blöcke bereit, die für den Aufbau eines DVB-T-Empfängers benötigt werden. Damit lässt sich ein vollständiger Empfänger aufbauen und der decodierte Transportstrom an einen UDP-Port ausgeben. Obwohl der Empfang grundsätzlich möglich war, ist er doch nicht zuverlässig genug für eine kontinuierliche Videowiedergabe.

## Besondere Anforderungen im Amateurfunk

Anders als beim herkömmlichen Fernsehen, bei dem ein Empfänger normalerweise auf ein dauerhaft vorhandenes Signal abgestimmt bleibt und eine kurze Wartezeit beim Programmwechsel akzeptiert wird, ist der Amateurfunkbetrieb wesentlich dynamischer. DATV-Aussendungen können nur kurz dauern, die sendenden Stationen können häufig wechseln und ein Signal kann unerwartet ausfallen und wiederkehren. Auch kann jederzeit eine andere Station auf derselben Frequenz zu senden beginnen.

Ein für den Amateurfunk geeigneter DVB-T-Empfänger muss deshalb nicht nur bei vorhandenem Signal eine stabile Synchronisation aufrechterhalten. Er sollte auch möglichst schnell und zuverlässig auf ein eintreffendes Signal synchronisieren und nach einer Empfangsunterbrechung zügig wieder synchron werden. Lange Synchronisationszeiten oder wiederholte Fehlschläge bei der Synchronisation können den Empfang selbst grundsätzlich empfangbarer Aussendungen unmöglich machen. Diese Anforderungen waren ein wesentlicher Beweggrund für die Entwicklung des Blocks **DVB-T OFDM Synchronization** in gr-dl5eu. Wie bei jedem DVB-T-Empfänger müssen die eingestellten Empfangsparameter natürlich zum eintreffenden Signal passen; eine schnelle Neusynchronisation bedeutet nicht, dass sich der Empfänger automatisch an jede Änderung der Übertragungsparameter anpasst.

## Entwicklungsgeschichte

### OFDM-Synchronisation

Eine wesentliche Schwierigkeit war die zuverlässige Erkennung der OFDM-Symbole. Während kleinere Abweichungen zwischen Sende- und Empfangsfrequenz während der Symbolerkennung korrigiert werden führen Unterschiede zwischen den Abtastraten von Sender und Empfänger dazu, dass sich die Position des Cyclic Prefix (Guardintervalls) relativ zum Abtastfenster des Empfängers verschiebt. Ohne ausreichende Korrektur der Abtastrate und Nachführung der Zeitlage kann die Symbolerkennung irgendwann die Synchronisation verlieren. Die anschließende Neusynchronisation unterbricht den decodierten Videostream.

Ein wichtiger Ausgangspunkt für Verbesserungen war das Projekt [`gr-isdbt`](https://github.com/git-artes/gr-isdbt) von Federico „Larroca“ La Rocca und weiteren Mitwirkenden. Dessen Block **OFDM Synchronization** vereint die Erkennung der OFDM-Symbole, die FFT-Verarbeitung und Synchronisationskorrekturen, einschließlich der Kompensation von Abweichungen des Abtasttakts. Das Projekt enthält außerdem einen **TMCC Decoder**, der die Übertragungsparameter von ISDB-T decodiert.

Auf Grundlage dieser Blöcke habe ich entsprechende Blöcke für DVB-T entwickelt. Die OFDM-Synchronisation und die Dekodierung der Übertragungsparameter mussten an die unterschiedlichen Signalstrukturen von DVB-T und ISDB-T angepasst werden, insbesondere an ihre Pilotträger und Signalisierungsverfahren. DVB-T verwendet Transmission Parameter Signalling (TPS) anstelle des bei ISDB-T eingesetzten TMCC-Verfahrens.

### Reed-Solomon-Decodierung und Energy Descrambling

Weitere Empfangsversuche zeigten ein zusätzliches Problem am Ende der Dekoderkette. Der standardmäßig in GNU Radio enthaltene DVB-T-Reed-Solomon-Decoder kann Codewörter erkennen, deren Fehler nicht mehr korrigiert werden können. Die betroffenen MPEG-Transportpakete werden für die nachfolgenden Verarbeitungsstufen jedoch nicht ausdrücklich als fehlerhaft gekennzeichnet. Dadurch können beschädigte Paketdaten den standardmäßigen Energy Descrambler passieren, ohne dass die entsprechende Fehlerkennzeichnung (TEI) im MPEG-Transportstrom gesetzt wird. Dies kann schwer erklärbare Wiedergabestörungen oder das Einfrieren des Videos zur Folge haben.

Um die Fehlerbehandlung zu verbessern, stellt **gr-dl5eu** zwei aufeinander abgestimmte, modifizierte Blöcke bereit:

1. **DVB-T Reed-Solomon Decoder** korrigiert Fehler, soweit dies möglich ist, und fügt ein Stream-Tag `rs_uncorrectable` hinzu, das nicht korrigierbare Codewörter kennzeichnet.
2. **DVB-T Energy Descramble** führt das übliche Energy Descrambling durch und verwendet diese Information, um im zugehörigen 188-Byte-Transportpaket das MPEG-TS-Bit `transport_error_indicator` (TEI) zu setzen.

Das TEI-Bit kennzeichnet ein fehlerhaftes Paket nur, repariert dessen Inhalt aber nicht. Andere Empfangsprobleme können weiterhin zu Unterbrechungen führen. Durch diese Änderung wird jedoch verhindert, dass nicht korrigierbare Reed-Solomon-Fehler diese Verarbeitungsstufe ohne entsprechende Fehlerkennzeichnung im Transportstrom passieren.

**Für den Aufbau eines Empfängers mit gr-dl5eu wird dringend empfohlen, die beiden Blöcke DVB-T Reed-Solomon Decoder und DVB-T Energy Descramble aus diesem Modul gemeinsam anstelle der entsprechenden Standardblöcke von GNU Radio zu verwenden.**

## Von gr-dl5eu bereitgestellte Blöcke

### Hauptblöcke des Moduls

| Block | Funktion |
| --- | --- |
| **DVB-T OFDM Synchronization** | Erkennt und verfolgt DVB-T-OFDM-Symbole, führt die FFT-Verarbeitung durch und kompensiert Abweichungen der Frequenz und Abtastrate. |
| **DVB-T TPS Decoder** | Dekodiert die DVB-T-Übertragungsparameter (Transmission Parameter Signalling) und extrahiert die Nutzdatenträger für die weitere Verarbeitung. |
| **DVB-T Reed-Solomon Decoder** | Führt die äußere Vorwärtsfehlerkorrektur durch und kennzeichnet nicht korrigierbare Codewörter mittels Stream-Tags. |
| **DVB-T Energy Descramble** | Macht die DVB-T-Energieverwürfelung rückgängig und setzt das TEI-Bit für Pakete, die als nicht korrigierbar erkannt wurden. |

Die Blöcke OFDM Synchronization und TPS Decoder unterstützen die DVB-T-Übertragungsmodi **2K und 8K**, auch wenn bei niedrigen Bandbreiten nur der 2K-Modus sinnvoll ist.

### Zusätzliche Diagnoseblöcke

Zwei weitere Blöcke wurden hauptsächlich für die Entwicklung, Fehlersuche und Analyse erstellt:

- **DVB-T Viterbi Decoder** — ein alternativer Viterbi-Decoder, der bei der Untersuchung der Decoderkette hilfreich ist. Ein eigener Viterbi-Decoder ist für einen herkömmlichen DVB-T-Empfänger normalerweise nicht erforderlich. Die aktuellen Beispiel-Flowgraphs verwenden allerdings die in diesem Modul enthaltene Version.
- **MPEG-TS Monitor** — ein transparenter Monitor für MPEG-Transportpakete mit 188 Byte Länge. Er kann Probleme bei der Synchronisation, Transportfehler und Unregelmäßigkeiten der Continuity Counter melden sowie Diagnosestatistiken in einer CSV-Datei aufzeichnen. Für den normalen Empfang ist er optional.

## Beispiel-Flowgraphs für den Empfang

Das Verzeichnis [`examples`](https://github.com/dl5eu/gr-dl5eu/tree/main/examples) enthält zwei Flowgraphs für GNU Radio Companion, die die Verwendung des Moduls demonstrieren:

| Flowgraph | Empfangshardware |
| --- | --- |
| [`dvbt_rx_rtlsdr.grc`](https://github.com/dl5eu/gr-dl5eu/blob/main/examples/dvbt_rx_rtlsdr.grc) | RTL-SDR-Stick |
| [`dvbt_rx_pluto.grc`](https://github.com/dl5eu/gr-dl5eu/blob/main/examples/dvbt_rx_pluto.grc) | ADALM-PLUTO-SDR |

Beide Beispiele bieten Bedienelemente, mit denen sich der Empfänger auf das gesendete Signal einstellen lässt. Dazu gehören **Kanalbandbreite, Guardintervall, Modulationsverfahren und Coderate** sowie empfängerspezifische Einstellungen wie Mittenfrequenz und Verstärkung. Es ist zu beachten, dass bei jeder Änderung eines dieser Parameter der Empfänger neu gestartet werden muss. Das Benutzerinterface stellt hierzu einen Restart-Button bereit.

Die Flowgraphs verwenden die GNU-Radio-Blöcke **Variable Config**, um ausgewählte Einstellungen in Konfigurationsdateien zu speichern (`config_rtlsdr.ini` beziehungsweise `config_pluto.ini`). Auf diese Weise können die Einstellungen beim nächsten Start des Empfängers abgerufen werden.

Der decodierte MPEG-Transportstrom wird an eine UDP Sink übergeben und kann mit VLC oder einem anderen Player wiedergegeben werden, der MPEG-TS über UDP empfangen kann.

Die Empfangseinstellungen müssen natürlich mit den Parametern des gesendeten DVB-T-Signals übereinstimmen.

## Praktische Einschränkungen und Testergebnisse

### RTL-SDR

Obwohl RTL-SDR-Geräte nominell Abtastraten von mehr als 2 MS/s liefern können, ist ein verlustfreier Empfang nicht in jeder Hardware- und Softwarekonfiguration gewährleistet. Es können Samples verloren gehen, ohne dass der Treiber eine entsprechende Warnung ausgibt. Selbst gelegentliche Sampleverluste können die OFDM-Synchronisation stören und sichtbare Unterbrechungen im Video verursachen.

Bei meinen Tests waren die mir zur Verfügung stehenden RTL-SDR-Sticks **gerade noch in der Lage, DVB-T-Signale mit 1 MHz Kanalbandbreite zu empfangen**. Kurze Aussetzer und gelegentliche Neusynchronisationen traten dennoch auf. Diese Ergebnisse lassen sich nicht ohne Weiteres auf alle RTL-SDR-Modelle oder einzelne Exemplare übertragen: die Leistungsfähigkeit hängt unter anderem vom Stick, der USB-Verbindung, dem Computer, dem Treiber und den Betriebsbedingungen ab.

In diesem anwendungsfall arbeitet ein RTL-SDR nahe an seinen praktischen Grenzen. Ein dauerhaft zuverlässiger Empfang kann nicht garantiert werden.

### Einfluss der Sendesignalqualität

Die Empfangsqualität hängt auch wesentlich von der Qualität des ausgesendeten DVB-T-Signals ab. Mit einem Sender auf Basis eines **HiDes-Chipsatzes** funktionierte der Empfang mit einem RTL-SDR unter den Testbedingungen recht gut.

Im Vergleich dazu führte ein DVB-T-Signal, das mit einem **Raspberry Pi 4B**, der **Portsdown-Software** und einem **ADALM-PLUTO** erzeugt wurde, bei gleichen DVB-T-Übertragungsparametern zu häufigeren Unterbrechungen des Empfangs. Diese Beobachtung bezieht sich auf die konkret getesteten Aufbauten und sollte nicht als allgemeine Bewertung der beiden Senderplattformen verstanden werden.

### ADALM-PLUTO als Empfänger

Versuche mit einem **ADALM-PLUTO** als Empfangs-SDR waren erfolgreicher. Mit dieser Konfiguration war auch der Empfang eines DVB-T-Signals mit **2 MHz Kanalbandbreite** möglich. Ein SDR mit zuverlässigerer Sampleübertragung ist deshalb eine sinnvolle Alternative, wenn ein RTL-SDR keinen ausreichend stabilen Empfang ermöglicht.

## Entwicklungsumgebung

Das Modul ist als OOT-Modul für GNU Radio 3.10 mit C++, Python-Bindings und Blockdefinitionen für GNU Radio Companion aufgebaut. Entwicklung und Tests erfolgten hauptsächlich mit **GNU Radio 3.10.12 unter Windows 11 und RadioConda**.

Das Repository enthält ein CMake-Buildsystem und ein [Conda-Build-Rezept](https://github.com/dl5eu/gr-dl5eu/tree/main/.conda/recipe). Andere Entwicklungsumgebungen und Hardwarekombinationen können zusätzliche Tests erfordern.

## Rückmeldungen und Mitarbeit

**gr-dl5eu befindet sich noch in der Entwicklung (work-in-progress).** Die Software kann sich im Verlauf der Tests weiter verändern, wenn zusätzliche Probleme oder Verbesserungsmöglichkeiten erkannt werden.

Ich würde mich besonders über Rückmeldungen von Funkamateuren freuen, die den Empfang schmalbandiger DVB-T-Signale mit diesem Modul und den Beispiel-Flowgraphs ausprobieren. Besonders hilfreich sind Angaben zur verwendeten SDR-Hardware, zu den Übertragungsparametern, zur Empfangsstabilität und zu eventuell beobachteten Problemen.

## Danksagung

Die Entwicklung der OFDM-Synchronisation und der TPS-Decodierung wurde durch Ideen und Implementierungen aus [`gr-isdbt`](https://github.com/git-artes/gr-isdbt) von **Federico „Larroca“ La Rocca und weiteren Mitwirkenden** angeregt und darauf aufgebaut. Dank gilt auch den Entwicklern von [GNU Radio](https://github.com/gnuradio/gnuradio), deren bestehende DVB-T-Blöcke die Grundlage des Empfängers bilden.

## Autor

**Ralf Gorholt (DL5EU)**  
E-Mail: [dl5eu@darc.de](mailto:dl5eu@darc.de)

## Lizenz

Dieses Projekt wird unter der **GNU General Public License, Version 3 oder höher (GPL-3.0-or-later)** veröffentlicht. Weitere Informationen enthält die Datei [LICENSE](https://github.com/dl5eu/gr-dl5eu/blob/main/LICENSE) im Repository.
