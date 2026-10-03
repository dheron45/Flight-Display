# Updater for the flight display. Runs at every hard reset, then starts the
# display (app.py, which loads flight_app.mpy).
#
# 1. If the display has crashed CRASH_LIMIT times in a row since its last good
#    run, restores the previous version and skips the one that crashed.
# 2. Fetches manifest.json from update_manifest_url (the repo's "dist" branch,
#    built by GitHub Actions for each release). If its version is new,
#    downloads each file to <name>.new, checks its size and CRC, then swaps them
#    in, keeping the old files as <name>.prev.
# 3. Starts the display in a fresh run, so it gets all of the board's memory.
#
# Any failure (no WiFi, a bad download...) just starts the current version.
#
# This file is installed once over USB and never updated over the air, so a bad
# release can't break the updater. Needs boot.py to make the drive writable.
#
# settings.toml:
#   CIRCUITPY_WIFI_SSID, CIRCUITPY_WIFI_PASSWORD (as for the display)
#   update_manifest_url = "https://raw.githubusercontent.com/dheron45/Flight-Display/dist/manifest.json"

import binascii
import json
import os
import time

import board
import busio
import microcontroller
import storage
import supervisor
from digitalio import DigitalInOut
from microcontroller import watchdog as w
from watchdog import WatchDogMode

# Same as the display, so a stuck download resets the board
w.timeout = 16  # seconds
w.mode = WatchDogMode.RESET

APP_FILE = "app.py"
VERSION_FILE = "/version.txt"
SKIP_FILE = "/skip_version.txt"
# Byte of microcontroller.nvm counting launches since the display last ran
# properly (bytes 0-3 hold the display's button settings). The display sets it
# back to 0 once it has fetched flights.
CRASH_COUNT_BYTE = 4
CRASH_LIMIT = 3
CHUNK_SIZE = 1024
# Files a release may never replace
PROTECTED = ("boot.py", "code.py", APP_FILE, "settings.toml", "secrets.py")


def read_text(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return ""


def write_text(path, text):
    with open(path, "w") as f:
        f.write(text)


def exists(path):
    try:
        os.stat(path)
        return True
    except OSError:
        return False


def remove(path):
    try:
        os.remove(path)
    except OSError:
        pass


# Rename, replacing the destination if there is one (FAT won't overwrite)
def replace(src, dst):
    remove(dst)
    os.rename(src, dst)


def crash_count():
    count = microcontroller.nvm[CRASH_COUNT_BYTE]
    return 0 if count == 0xFF else count  # 0xFF: never written


def set_crash_count(count):
    count = min(count, 0xFE)
    if microcontroller.nvm[CRASH_COUNT_BYTE] != count:
        microcontroller.nvm[CRASH_COUNT_BYTE] = count


def prev_files():
    return [name for name in os.listdir("/") if name.endswith(".prev")]


# Put back the files the last update replaced, and remember the version that
# kept crashing so it isn't installed again
def roll_back():
    bad_version = read_text(VERSION_FILE)
    print("Display crashed " + str(CRASH_LIMIT) + " times, rolling back from " + (bad_version or "?"))
    for name in prev_files():
        replace("/" + name, "/" + name[:-len(".prev")])
    if bad_version:
        write_text(SKIP_FILE, bad_version)
    print("Rolled back to " + (read_text(VERSION_FILE) or "?"))


def connect_wifi(esp):
    from adafruit_esp32spi import adafruit_esp32spi
    ssid = os.getenv("CIRCUITPY_WIFI_SSID")
    password = os.getenv("CIRCUITPY_WIFI_PASSWORD")
    for attempt in range(3):
        w.feed()
        try:
            esp.connect_AP(ssid, password, timeout_s=10)
            return True
        except (RuntimeError, OSError, ConnectionError) as e:
            print("WiFi connect failed: " + repr(e))
        w.feed()
        if esp.status == adafruit_esp32spi.WL_CONNECTED:
            return True
    return False


# Path of a manifest file on the board, or None if it isn't safe to write
def local_path(path):
    if not path or path.startswith("/") or ".." in path or path in PROTECTED:
        return None
    return "/" + path


# Stream url into dest, returning (bytes written, CRC32)
def download(requests, url, dest):
    size = 0
    crc = 0
    with requests.get(url) as response:
        if response.status_code != 200:
            raise OSError("HTTP " + str(response.status_code) + " for " + url)
        with open(dest, "wb") as f:
            for chunk in response.iter_content(chunk_size=CHUNK_SIZE):
                f.write(chunk)
                size += len(chunk)
                crc = binascii.crc32(chunk, crc)
                w.feed()
    return size, crc & 0xFFFFFFFF


def install(requests, manifest, base_url):
    files = manifest["files"]
    paths = [local_path(entry["path"]) for entry in files]
    if None in paths:
        raise ValueError("Manifest has a file the updater won't write")

    stats = os.statvfs("/")
    free = stats[0] * stats[4]
    needed = sum(entry["size"] for entry in files) + 4096
    if free < needed:
        raise OSError("Not enough space: need " + str(needed) + ", have " + str(free))

    try:
        for entry, path in zip(files, paths):
            print("Downloading " + entry["path"])
            size, crc = download(requests, base_url + entry["path"], path + ".new")
            if size != entry["size"] or crc != entry["crc32"]:
                raise ValueError(entry["path"] + " doesn't match the manifest (size "
                                 + str(size) + ", CRC " + hex(crc) + ")")
    except Exception:
        for path in paths:
            remove(path + ".new")
        raise

    # Everything downloaded and checked: swap it in, keeping the old files
    for name in prev_files():
        remove("/" + name)
    for path in paths:
        if exists(path):
            os.rename(path, path + ".prev")
        os.rename(path + ".new", path)
    if exists(VERSION_FILE):
        os.rename(VERSION_FILE, VERSION_FILE + ".prev")
    write_text(VERSION_FILE, manifest["version"])
    remove(SKIP_FILE)


def check_for_update():
    manifest_url = os.getenv("update_manifest_url")
    if not manifest_url:
        print("No update_manifest_url in settings.toml, skipping update check")
        return False

    import adafruit_connection_manager
    import adafruit_requests
    from adafruit_esp32spi import adafruit_esp32spi

    esp32_cs = DigitalInOut(board.ESP_CS)
    esp32_ready = DigitalInOut(board.ESP_BUSY)
    esp32_reset = DigitalInOut(board.ESP_RESET)
    spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
    esp = adafruit_esp32spi.ESP_SPIcontrol(spi, esp32_cs, esp32_ready, esp32_reset)
    if not connect_wifi(esp):
        print("No WiFi, skipping update check")
        return False

    pool = adafruit_connection_manager.get_radio_socketpool(esp)
    ssl_context = adafruit_connection_manager.get_radio_ssl_context(esp)
    requests = adafruit_requests.Session(pool, ssl_context)

    w.feed()
    with requests.get(manifest_url) as response:
        if response.status_code != 200:
            raise OSError("HTTP " + str(response.status_code) + " fetching manifest")
        manifest = response.json()
    version = manifest["version"]
    installed = read_text(VERSION_FILE)
    print("Installed version " + (installed or "none") + ", latest " + version)
    if version == installed:
        return False
    if version == read_text(SKIP_FILE):
        print("Skipping " + version + ": it crashed and was rolled back")
        return False

    install(requests, manifest, manifest_url[:manifest_url.rfind("/") + 1])
    print("Installed " + version)
    return True


def start_display():
    try:
        w.deinit()
    except Exception:
        pass  # the display sets the watchdog up again straight away anyway
    supervisor.set_next_code_file(APP_FILE)
    supervisor.reload()


if storage.getmount("/").readonly:
    # boot.py left the drive writable over USB (UP held at reset)
    print("USB edit mode: the drive is read-only to the board, skipping updates")
else:
    count = crash_count()
    if count >= CRASH_LIMIT and prev_files():
        roll_back()
        count = 0
    try:
        if check_for_update():
            count = 0
    except Exception as e:
        print("Update failed, starting the current version: " + repr(e))
    set_crash_count(count + 1)

start_display()
