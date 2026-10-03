# Runs once at every hard reset, before code.py.
#
# Makes the CIRCUITPY drive writable by the board's own code, so the updater
# (code.py) can install new versions downloaded from GitHub. While it's writable
# by the board, the computer can only read it.
#
# To edit files over USB instead: hold the UP button while pressing RESET.
# The drive is then writable from the computer, and updates are skipped.

import board
import storage
from digitalio import DigitalInOut, Pull

up = DigitalInOut(board.BUTTON_UP)
up.pull = Pull.UP
usb_edit_mode = not up.value  # the button reads False while pressed
up.deinit()

if not usb_edit_mode:
    storage.remount("/", readonly=False)
