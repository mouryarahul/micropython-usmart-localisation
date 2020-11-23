import time
import pyb
from machine import RTC
from uac_localisation.main.misc.datetime import datetime

def delta_time(ts1: tuple, ts2: tuple):
    """
    calculates difference between two time stamps.
    :param ts1: should be tuple (micros, millis) given by pyb.millis() and pyb.micros()
    :param ts2: should be tuple (micros, millis) given by pyb.millis() and pyb.micros()
    :return: float containing time difference (ts2-ts1) in seconds
    Note: pyb.millis and pyb.millis counter rollover to zero at 0x3fffffff counts
    """
    delta_millis = time.ticks_diff(ts2[1], ts1[1])
    delta_micros = time.ticks_diff(ts2[0], ts1[0])
    delta_micros = delta_micros % 1000  # in micro seconds
    delta_millis = delta_micros/1E6 + delta_millis/1E3
    return delta_millis

# Initialize RTC
rtc = RTC()
delta = 10001110
for i in range(4):
    micro_start = pyb.micros()
    milli_start = pyb.millis()

    pyb.udelay(delta)

    micro_end = pyb.micros()
    milli_end = pyb.millis()
    delta_millis = delta_time((micro_start, milli_start), (micro_end, milli_end))
    print("Delta Time by SysTick:  ", delta_millis)

    rtc.silent_read()
    usecond_start = rtc.useconds()
    second_start = rtc.seconds()
    minute_start = rtc.minutes()
    hour_start = rtc.hours()
    weekday_start = rtc.weekday()
    date_start = rtc.date()
    month_start = rtc.month()
    year_start = rtc.year()

    pyb.udelay(delta)

    rtc.silent_read()
    usecond_end = rtc.useconds()
    second_end = rtc.seconds()
    minute_end = rtc.minutes()
    hour_end = rtc.hours()
    weekday_end = rtc.weekday()
    date_end = rtc.date()
    month_end = rtc.month()
    year_end = rtc.year()
    dt_start = (year_start, month_start, date_start, weekday_start, hour_start, minute_start, second_start, usecond_start)
    dt_end = (year_end, month_end, date_end, weekday_end, hour_end, minute_end, second_end, usecond_end)
    dt_start = datetime.fromrtctodatetime(dt_start)
    dt_end = datetime.fromrtctodatetime(dt_end)
    print("Delta Time by RTC: \t", (dt_end-dt_start).total_seconds())
