# Central receiver-bandwidth configuration. Edit only these entries when
# changing the available bandwidths; update the matching GRC resampler ports.
PLUTO_SAMPLE_RATE = 2400000
BANDWIDTHS = (
    ("2 MHz", 20, 21),
    ("1 MHz", 10, 21),
    ("500 kHz", 5, 21),
    ("333 kHz", 160, 1009),
    ("250 kHz", 5, 42),
    ("125 kHz", 5, 84),
    ("66 kHz", 280, 8909),
)


def interpolation(index):
    return BANDWIDTHS[int(index)][1]


def decimation(index):
    return BANDWIDTHS[int(index)][2]


def sample_rate(index):
    return PLUTO_SAMPLE_RATE * interpolation(index) / decimation(index)


def labels():
    return [entry[0] for entry in BANDWIDTHS]


def indices():
    return list(range(len(BANDWIDTHS)))