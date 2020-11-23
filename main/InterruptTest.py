# Testing for Switch Interrupt
import micropython
import pyb
from pyb import RTC
from pyb import Switch

micropython.alloc_emergency_exception_buf(200)

rtc = RTC()
sw = Switch()
print(rtc.datetime())

# Time stamp variables
hours = 0
minutes = 0
seconds = 0
useconds = 0


def sw_callback():
    global hours
    global minutes
    global seconds
    global useconds
    # Read RTC silently
    rtc.silent_read()
    hours = rtc.hours()
    minutes = rtc.minutes()
    seconds = rtc.seconds()
    useconds = rtc.useconds()
    print(useconds)
    print(seconds)
    print(minutes)

# Setup switch callback
sw.callback(sw_callback)

while True:
    pyb.delay(1000)
