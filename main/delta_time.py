import time

def delta_time(ts1: tuple, ts2: tuple):
    """
    calculates difference between two time stamps.
    :param ts1: should be tuple (micros, millis, seconds) given by time.time(), pyb.millis() and pyb.micros()
    :param ts2: should be tuple (micros, millis, seconds) given by time.time(), pyb.millis() and pyb.micros()
    :return: float containing time difference (ts2-ts1) in seconds
    Note: pyb.millis and pyb.millis counter rollover to zero at 0x3fffffff counts
    """
    delta_seconds = ts2[2] - ts1[2]
    delta_millis = time.ticks_diff(ts2[1], ts1[1])
    delta_micros = time.ticks_diff(ts2[0], ts1[0])

    delta_micros = delta_micros % 1000  # in micro seconds
    delta_millis = delta_millis % 1000  # in milli seconds
    delta_seconds = delta_seconds + delta_micros/1E6 + delta_millis/1E3
    return delta_seconds
