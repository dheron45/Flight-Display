# Starts the flight display (flight_app.mpy, installed by the updater in code.py).
#
# If the display code crashes, the board is reset so it goes back through the
# updater, which can install a fix, or roll back to the previous version after
# repeated crashes.

import time
import traceback

import microcontroller

try:
    import flight_app
except Exception as e:
    traceback.print_exception(e)
    print("Display crashed - resetting in 10 seconds")
    time.sleep(10)
    microcontroller.reset()
