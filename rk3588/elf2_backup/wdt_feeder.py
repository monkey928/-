#!/usr/bin/env python3
"""Watchdog feeder - feed every 5 seconds"""
import time, sys

with open("/dev/watchdog0", "w") as wdt:
    sys.stdout.write("WDT feeder started
")
    sys.stdout.flush()
    while True:
        wdt.write("0")
        wdt.flush()
        time.sleep(5)
