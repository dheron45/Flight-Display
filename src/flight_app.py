# Overhead flight display for the Adafruit Matrix Portal M4 + 64x32 RGB LED matrix.
#
# Every QUERY_DELAY seconds it finds the closest aircraft flying within the
# search range (1, 3 or 5 nautical miles) of your location, then shows:
#   Airline flights (3 lines):  origin > destination airports
#                               flight number, in the airline's colour
#                               aircraft type
#   Private/light aircraft (2 lines): aircraft model, then altitude
# Each line shows short text first, then scrolls a longer version (e.g. the full
# airport names or airline name).
#
# Buttons: UP turns a clock on/off, shown whenever there are no flights.
#          UP pressed twice quickly cycles through three brightness levels.
#          DOWN cycles the search range through RANGE_OPTIONS_NM.
# These settings are remembered after a restart.
#
# Data comes from free APIs that need no key (see "Data sources" below).
#
# Setup: put these in settings.toml on the CIRCUITPY drive (never commit that file):
#   CIRCUITPY_WIFI_SSID = "your network"
#   CIRCUITPY_WIFI_PASSWORD = "your password"
#   home_lat = "51.5074"    (example: central London - use your own location.
#   home_lon = "-0.1278"     Keep the quotes: settings.toml can't store decimals.)
#   update_manifest_url = "https://raw.githubusercontent.com/dheron45/Flight-Display/dist/manifest.json"
#
# Deployment: this file is compiled to flight_app.mpy for each GitHub release and
# installed over WiFi by the updater (code.py on the board) - see README.md.
# Every UPDATE_CHECK_SECONDS it checks for a new release, and resets the board
# so the updater can install it.

import gc
import math
import os
import time

import keypad
import rtc
import supervisor

import board
import busio
import displayio
import neopixel
import terminalio
from digitalio import DigitalInOut

import adafruit_connection_manager
import adafruit_requests
import adafruit_display_text.label
from adafruit_esp32spi import adafruit_esp32spi
from adafruit_esp32spi import adafruit_esp32spi_wifimanager
from adafruit_matrixportal.matrix import Matrix

import microcontroller
from microcontroller import watchdog as w
from watchdog import WatchDogMode

# CircuitPython normally restarts the code whenever anything is written to the
# CIRCUITPY drive - including background writes by the computer it's plugged into
# (e.g. macOS indexing), which makes the display restart at random. Set to True
# while editing code.py on the board, so saving it restarts the display straight
# away; with False, press the board's RESET button (or Ctrl-D in the serial
# console) after saving changes.
AUTO_RELOAD=False
supervisor.runtime.autoreload = AUTO_RELOAD

# Reset the board if the code hangs (e.g. a stuck network request) for longer
# than this. Long-running loops call w.feed() to show they're still alive.
w.timeout=16 # seconds
w.mode = WatchDogMode.RESET

FONT=terminalio.FONT

WIFI_SSID=os.getenv("CIRCUITPY_WIFI_SSID")
WIFI_PASSWORD=os.getenv("CIRCUITPY_WIFI_PASSWORD")
SEARCH_LAT=os.getenv("home_lat")
SEARCH_LON=os.getenv("home_lon")
if not (WIFI_SSID and SEARCH_LAT and SEARCH_LON):
    raise RuntimeError("Add CIRCUITPY_WIFI_SSID, CIRCUITPY_WIFI_PASSWORD, home_lat and home_lon to settings.toml")


# --- Data sources ---
#
# 1. Aircraft nearby: adsb.fi (opendata.adsb.fi) - every aircraft within a radius
#    of a point, with position, altitude, speed, registration and aircraft type.
# 2. Route: adsb.im's routeset API (the one the tar1090 map uses). We send the
#    callsign AND the aircraft's live position, and it says whether its route for
#    that callsign is "plausible" for where the plane actually is. Airlines reuse
#    callsigns for different routes, so callsign-only route databases are often
#    wrong; this is much more reliable. route_is_sane() adds our own check on top.
# 3. Airline name: the AIRLINE_NAMES table below, falling back to adsbdb.com's
#    airline lookup for anything not in it (cached, so once per airline).

# How often to check for aircraft, in seconds
QUERY_DELAY=15

# Search ranges (radius in nautical miles) the DOWN button cycles through; the
# first is used until you change it. adsb.fi allows up to 250, but keep them
# small - that suits "what's overhead", and keeps the response small enough for
# the board's limited memory.
RANGE_OPTIONS_NM=(1,3,5)

FLIGHT_SEARCH_HEAD="https://opendata.adsb.fi/api/v3/lat/"+str(SEARCH_LAT)+"/lon/"+str(SEARCH_LON)+"/dist/"

# How often to check GitHub for a new release, in seconds (the updater also
# checks at every reset)
UPDATE_CHECK_SECONDS=60*60
UPDATE_MANIFEST_URL=os.getenv("update_manifest_url")

# Set to False to skip route lookups (line 1 then shows the callsign instead)
LOOKUP_ROUTES=True
ROUTE_LOOKUP_URL="https://adsb.im/api/0/routeset"
AIRLINE_LOOKUP_HEAD="https://api.adsbdb.com/v0/airline/"

# A route is rejected if the plane is cruising (more than ROUTE_CHECK_MIN_KM from
# both airports) but heading more than ROUTE_MAX_HEADING_OFF degrees away from
# the destination - it's clearly not flying that route. Close to either airport
# planes climb out, hold and turn in all directions, so there's no check there.
ROUTE_CHECK_MIN_KM=150
ROUTE_MAX_HEADING_OFF=90

# Airlines by their 3-letter ICAO code (the first three letters of the callsign,
# e.g. BAW615 -> BAW), giving:
#   - the name people know (e.g. "KLM" rather than "KLM Royal Dutch Airlines")
#   - the 2-character IATA code used for flight numbers (e.g. RYR3N -> FR3N)
#   - a brand colour for line 2 and the plane icon
# Airlines not listed are looked up from adsbdb (formal names, no colour).
# Colours are brightened versions of each brand colour, as LED panels show dark
# shades like navy very dimly.
AIRLINE_NAMES={
    # UK and European
    "BAW": ("British Airways","BA",0x2A6BE0), "SHT": ("British Airways","BA",0x2A6BE0),
    "EFW": ("British Airways","BA",0x2A6BE0), "CFE": ("BA CityFlyer","BA",0x2A6BE0),
    "EZY": ("easyJet","U2",0xFF6600), "EJU": ("easyJet","EC",0xFF6600), "EZS": ("easyJet","DS",0xFF6600),
    "RYR": ("Ryanair","FR",0xF1C933), "RUK": ("Ryanair","RK",0xF1C933), "MAY": ("Ryanair","FR",0xF1C933),
    "WZZ": ("Wizz Air","W6",0xE0008A), "WUK": ("Wizz Air","W9",0xE0008A), "WMT": ("Wizz Air","W4",0xE0008A),
    "EXS": ("Jet2","LS",0xE30613), "TOM": ("TUI","BY",0x40C0F0), "VIR": ("Virgin Atlantic","VS",0xE10A0A),
    "KLM": ("KLM","KL",0x00A1DE), "IBE": ("Iberia","IB",0xE8192D), "VLG": ("Vueling","VY",0xFFCC00),
    "TAP": ("TAP Air Portugal","TP",0x3DAE2B), "SAS": ("SAS","SK",0x3A6FE0), "SWR": ("Swiss","LX",0xE2001A),
    "NSZ": ("Norwegian","D8",0xE01E3C), "NOZ": ("Norwegian","DY",0xE01E3C), "NAX": ("Norwegian","DY",0xE01E3C),
    "AUR": ("Aurigny","GR",0xFFD200), "TVS": ("Smartwings","QS",0x0096D6),
    "DLH": ("Lufthansa","LH",0xFFAD00), "AFR": ("Air France","AF",0xE4002B),
    "EIN": ("Aer Lingus","EI",0x00B08A), "EWG": ("Eurowings","EW",0xD0206A),
    "AUA": ("Austrian","OS",0xE0001B), "FIN": ("Finnair","AY",0x4070F0),
    "THY": ("Turkish Airlines","TK",0xE81932), "LOG": ("Loganair","LM",0x4070F0),
    # Long haul
    "UAE": ("Emirates","EK",0xE8202A), "QTR": ("Qatar Airways","QR",0xB0306A),
    "ETD": ("Etihad","EY",0xD4A017), "SIA": ("Singapore Airlines","SQ",0xFCB130),
    "AAL": ("American Airlines","AA",0x2090F0), "UAL": ("United Airlines","UA",0x2A7FE0),
    "DAL": ("Delta","DL",0xE0102E), "ACA": ("Air Canada","AC",0xE02630),
    "JBU": ("JetBlue","B6",0x2E6FDB),
    # Cargo
    "UPS": ("UPS","5X",0xFFB500), "FDX": ("FedEx","FX",0xFF6200),
    "DHK": ("DHL","D0",0xFFCC00), "BCS": ("DHL","QY",0xFFCC00),
}
# Airlines already looked up from adsbdb since startup, so each is only fetched once
airline_cache={}

# --- Colours ---
# Line 1 when there's no route (callsign, or the model for private aircraft)
ROW_ONE_COLOUR=0xEE82EE
# Line 1 route: origin airport, destination airport and the ">" between them
ORIGIN_COLOUR=0xEE82EE
DESTINATION_COLOUR=0x40E0D0
ROUTE_ARROW_COLOUR=0x909090
# Line 2 for airlines not in AIRLINE_NAMES
ROW_TWO_COLOUR=0x4B0082
# Line 2 (altitude) and plane icon for private/light aircraft
GA_ALTITUDE_COLOUR=0x40D040
# Line 3: soft periwinkle, between line 1's violet and turquoise and pale enough
# not to clash with whichever airline colour is on line 2
ROW_THREE_COLOUR=0x9A9CFF
# Clock shown when there are no flights
CLOCK_COLOUR=0x9A9CFF

# --- Timings ---
# Seconds to pause between scrolling one line and the next
PAUSE_BETWEEN_LABEL_SCROLLING=2
# Seconds per pixel step - lower is faster
PLANE_SPEED=0.03
TEXT_SPEED=0.03

# --- Layout ---
# Characters that fit across the panel without scrolling (6 pixels each)
MAX_STATIC_CHARS=10
# Vertical positions of the lines, so each layout sits in the middle of the panel
THREE_LINE_Y=(4,15,25)
TWO_LINE_Y=(9,21)
# Gap in pixels either side of the route's ">" (one character)
ROUTE_GAP=6

# --- Clock and brightness ---
# Whether the clock is on the first time the display is started. After that the
# UP button's setting is remembered (see "Saved settings").
CLOCK_ON_AT_START=False
# Your time zone's offset from UTC in hours, not counting summer time (UK: 0)
UTC_OFFSET_HOURS=0
# Add an hour from the last Sunday in March to the last Sunday in October, as
# in the UK and EU. Set to False if your time zone has no summer time.
EU_SUMMER_TIME=True
# How often to re-sync the clock with internet time (taken from API responses), in seconds
CLOCK_RESYNC_SECONDS=6*60*60
# Brightness levels a double press of UP cycles through (1.0 = full brightness)
BRIGHTNESS_LEVELS=(1.0,0.5,0.2)
# Two UP presses within this many milliseconds count as a double press. A single
# press waits this long before toggling the clock, in case a second one follows.
DOUBLE_PRESS_MS=400
# How long a setting change message (e.g. "Range 3nm") stays on the panel, in seconds
MESSAGE_SECONDS=1.5

# Headers sent with every request. The browser-style User-Agent is carried over
# from the original version of this script.
rheaders = {
     "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64; rv:106.0) Gecko/20100101 Firefox/106.0",
     "cache-control": "no-store, no-cache, must-revalidate, post-check=0, pre-check=0",
     "accept": "application/json"
}

# Errors from a network request that we recover from by retrying/reconnecting
# (ValueError covers a response that isn't valid JSON)
NETWORK_ERRORS=(RuntimeError, OSError, ValueError, adafruit_requests.OutOfRetries)


# --- Hardware setup ---

# The Matrix Portal's WiFi is a separate ESP32 co-processor ("AirLift") on SPI
esp32_cs = DigitalInOut(board.ESP_CS)
esp32_ready = DigitalInOut(board.ESP_BUSY)
esp32_reset = DigitalInOut(board.ESP_RESET)
spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
esp = adafruit_esp32spi.ESP_SPIcontrol(spi, esp32_cs, esp32_ready, esp32_reset)

pool = adafruit_connection_manager.get_radio_socketpool(esp)
ssl_context = adafruit_connection_manager.get_radio_ssl_context(esp)
requests = adafruit_requests.Session(pool, ssl_context)

# The on-board status LED shows the WiFi connection state
status_light = neopixel.NeoPixel(board.NEOPIXEL, 1, brightness=0.2)
wifi = adafruit_esp32spi_wifimanager.WiFiManager(esp, WIFI_SSID, WIFI_PASSWORD, status_pixel=status_light, debug=False, attempts=1)

# Just the LED matrix display - we do our own networking, so we don't need the
# full MatrixPortal helper (which also loads Adafruit IO, MQTT etc.)
# bit_depth=4 gives 16 shades per colour channel, enough for the dimmer
# brightness levels (the panel itself can only be fully on or off).
display=Matrix(width=64, height=32, bit_depth=4).display
DISPLAY_WIDTH=display.width

# The UP and DOWN buttons. keypad watches them in the background, so presses
# aren't missed while the display is busy scrolling.
buttons=keypad.Keys((board.BUTTON_UP, board.BUTTON_DOWN), value_when_pressed=False, pull=True)
BUTTON_UP=0
BUTTON_DOWN=1


# --- Saved settings ---
# The button settings are kept in the board's non-volatile memory
# (microcontroller.nvm), which survives reboots and power cuts. Layout:
#   byte 0: SETTINGS_MARKER, showing the settings below have been saved
#   byte 1: clock on (1) or off (0)
#   byte 2: brightness level (an index into BRIGHTNESS_LEVELS)
#   byte 3: search range (an index into RANGE_OPTIONS_NM)
SETTINGS_MARKER=0xA5

# (clock_on, brightness_level, range_index) as last saved. Any setting that
# hasn't been saved or no longer fits (e.g. RANGE_OPTIONS_NM has been shortened)
# gets its default instead.
def load_settings():
    try:
        marker,clock,level,range_i=microcontroller.nvm[0:4]
    except (AttributeError, TypeError, ValueError):  # no nvm on this board
        return CLOCK_ON_AT_START,0,0
    if marker!=SETTINGS_MARKER:
        return CLOCK_ON_AT_START,0,0
    return (clock==1 if clock<=1 else CLOCK_ON_AT_START,
            level if level<len(BRIGHTNESS_LEVELS) else 0,
            range_i if range_i<len(RANGE_OPTIONS_NM) else 0)

# Save the settings, only writing if they've changed (to spare the flash memory)
def save_settings():
    saved=bytes((SETTINGS_MARKER, 1 if clock_on else 0, brightness_level, range_index))
    try:
        if microcontroller.nvm[0:4]!=saved:
            microcontroller.nvm[0:4]=saved
    except (AttributeError, TypeError, ValueError):  # no nvm on this board
        pass

# Byte 4 of nvm is the updater's count of launches since the display last ran
# properly. Setting it to 0 tells the updater this version works, so it won't
# roll it back.
CRASH_COUNT_BYTE=4

def mark_healthy():
    try:
        if microcontroller.nvm[CRASH_COUNT_BYTE]!=0:
            microcontroller.nvm[CRASH_COUNT_BYTE]=0
    except (AttributeError, TypeError, ValueError, IndexError):  # no nvm on this board
        pass

clock_on,brightness_level,range_index=load_settings()
print("Saved settings: clock "+("on" if clock_on else "off")+", brightness "+str(BRIGHTNESS_LEVELS[brightness_level])
      +", range "+str(RANGE_OPTIONS_NM[range_index])+"nm")


# --- Brightness ---
# Brightness is done by dimming every colour, so each label's full-brightness
# colour is remembered here to re-apply when the level changes.

label_colours=[]  # [label, full-brightness colour] pairs
plane_colour=ROW_TWO_COLOUR

# A colour scaled to the current brightness level
def dim(colour):
    factor=BRIGHTNESS_LEVELS[brightness_level]
    r=int(((colour>>16)&0xFF)*factor)
    g=int(((colour>>8)&0xFF)*factor)
    b=int((colour&0xFF)*factor)
    return (r<<16)|(g<<8)|b

def set_label_colour(label, colour):
    for entry in label_colours:
        if entry[0] is label:
            entry[1]=colour
            break
    else:
        label_colours.append([label,colour])
    label.color=dim(colour)

def set_plane_colour(colour):
    global plane_colour
    plane_colour=colour
    planePalette[1]=dim(colour)

def next_brightness():
    global brightness_level
    brightness_level=(brightness_level+1)%len(BRIGHTNESS_LEVELS)
    print("Brightness "+str(BRIGHTNESS_LEVELS[brightness_level]))
    save_settings()
    for label,colour in label_colours:
        label.color=dim(colour)
    planePalette[1]=dim(plane_colour)


# --- Display objects ---

# Plane icon that flies across the panel when a new flight is found
PLANE_ART=(
    "......#.....",
    ".....##.....",
    "....###.....",
    "...###...#..",
    ".#########..",
    ".#########..",
    "...###...#..",
    "....###.....",
    ".....##.....",
    "......#.....",
)
planeBmp = displayio.Bitmap(12, 12, 2)
for y,row in enumerate(PLANE_ART):
    for x,pixel in enumerate(row):
        if pixel=="#":
            planeBmp[x,y]=1
planePalette = displayio.Palette(2)
planePalette[0] = 0x000000
planePalette[1] = dim(plane_colour)  # set per flight in plane_animation()
planeG=displayio.Group(x=DISPLAY_WIDTH+12,y=10)
planeG.append(displayio.TileGrid(planeBmp, pixel_shader=planePalette))

def make_label(colour, scale=1):
    label=adafruit_display_text.label.Label(FONT, text="", scale=scale)
    set_label_colour(label, colour)
    return label

# One label per line of text - display_flight() sets their text and position
label1 = make_label(ROW_ONE_COLOUR)
label2 = make_label(ROW_TWO_COLOUR)
label3 = make_label(ROW_THREE_COLOUR)

# A route on line 1 is three labels side by side (a label can only be one
# colour), grouped so they centre and scroll together
route_origin = make_label(ORIGIN_COLOUR)
route_arrow = make_label(ROUTE_ARROW_COLOUR)
route_dest = make_label(DESTINATION_COLOUR)
route_group = displayio.Group(y=THREE_LINE_Y[0])
route_group.append(route_origin)
route_group.append(route_arrow)
route_group.append(route_dest)

g = displayio.Group()
g.append(label1)
g.append(label2)
g.append(label3)
g.append(route_group)
display.root_group = g

# Clock, in double-size digits in the middle of the panel
clock_label = make_label(CLOCK_COLOUR, scale=2)
clock_label.anchor_point=(0.5,0.5)
clock_label.anchored_position=(DISPLAY_WIDTH//2, display.height//2)
clock_group = displayio.Group()
clock_group.append(clock_label)

# Short message shown when a setting changes, e.g. "Range 3nm"
message_label = make_label(ROW_ONE_COLOUR)
message_label.anchor_point=(0.5,0.5)
message_label.anchored_position=(DISPLAY_WIDTH//2, display.height//2)
message_group = displayio.Group()
message_group.append(message_label)


# --- Drawing ---

# Fly the plane icon right to left across the panel, in the flight's line 2 colour
def plane_animation(colour):
    set_plane_colour(colour)
    display.root_group = planeG
    for i in range(DISPLAY_WIDTH+24,-12,-1):
        planeG.x=i
        service()
        time.sleep(PLANE_SPEED)

# Scroll a label (or a group of labels, given its width) from just off the
# right edge of the panel until it has completely left the left edge
def scroll(line, width=None):
    if width is None:
        width=line.bounding_box[2]
    for i in range(DISPLAY_WIDTH+1,-width,-1):
        line.x=i
        service()
        time.sleep(TEXT_SPEED)

# Set a label's text and centre it on the panel. Text wider than the panel is
# left-aligned so its start stays visible.
def show_centered(line, text):
    line.text=text
    line.x=max(0,(DISPLAY_WIDTH-line.bounding_box[2])//2)

# Lay out "origin > destination" in the route labels and return its total width.
# With fit=True, the gaps around ">" are shrunk if they'd push it off the panel
# (e.g. with 4-letter ICAO codes). set_route("","") blanks the route, ">" included.
def set_route(origin, dest, fit=False):
    route_origin.text=origin
    route_arrow.text=">" if (origin or dest) else ""
    route_dest.text=dest
    widths=route_origin.bounding_box[2]+route_arrow.bounding_box[2]+route_dest.bounding_box[2]
    gap=ROUTE_GAP
    if fit and widths+2*gap>DISPLAY_WIDTH:
        gap=1
    route_origin.x=0
    route_arrow.x=route_origin.bounding_box[2]+gap
    route_dest.x=route_arrow.x+route_arrow.bounding_box[2]+gap
    return widths+2*gap

def show_route_centered(origin, dest):
    width=set_route(origin, dest, fit=True)
    route_group.x=max(0,(DISPLAY_WIDTH-width)//2)

# Show a flight described by parse_flight(): each line's short text, then the
# longer version of each line scrolled across in turn
def display_flight(flight):
    # Set everything up before switching the panel to it, so half-updated
    # labels (e.g. new airport code with the old ">" position) never show
    lines=(label1,label2,label3)
    for line,y in zip(lines, TWO_LINE_Y if flight["two_line"] else THREE_LINE_Y):
        line.y=y
    set_label_colour(label2, flight["line2_colour"])

    route=flight["route"]
    if route:
        show_route_centered(route[0],route[1])
    else:
        set_route("","")
    for line,(short_text,long_text) in zip(lines,flight["text"]):
        show_centered(line,short_text)
    display.root_group = g
    time.sleep(PAUSE_BETWEEN_LABEL_SCROLLING)

    if route:
        # scroll the full airport names, keeping their colours
        route_group.x=DISPLAY_WIDTH+1
        scroll(route_group, set_route(route[2],route[3]))
        show_route_centered(route[0],route[1])
        time.sleep(PAUSE_BETWEEN_LABEL_SCROLLING)

    for line,(short_text,long_text) in zip(lines,flight["text"]):
        # skip lines with nothing more to show (e.g. altitude, or a model that fits)
        if long_text in ("",short_text):
            continue
        line.x=DISPLAY_WIDTH+1
        line.text=long_text
        scroll(line)
        show_centered(line,short_text)
        time.sleep(PAUSE_BETWEEN_LABEL_SCROLLING)

# Blank the panel
def clear_flight():
    label1.text=label2.text=label3.text=""
    set_route("","")


# --- Clock and buttons ---

clock_synced=False
last_clock_sync=0
idle=True  # True when there's no flight to show (the clock's turn)

# UTC offset in seconds at the given UTC time, including EU/UK summer time
def utc_offset(utc_seconds):
    offset=UTC_OFFSET_HOURS*3600
    if not EU_SUMMER_TIME:
        return offset
    year=time.localtime(utc_seconds).tm_year
    # last Sunday of the month (tm_wday: Monday=0 ... Sunday=6), at 01:00 UTC
    def last_sunday(month):
        day31=time.localtime(time.mktime((year,month,31,1,0,0,0,-1,-1)))
        return time.mktime((year,month,31-(day31.tm_wday+1)%7,1,0,0,0,-1,-1))
    if last_sunday(3)<=utc_seconds<last_sunday(10):
        offset+=3600
    return offset

MONTHS=("Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec")

# Set the board's clock from a web server's "Date" response header, e.g.
# "Sun, 27 Sep 2026 12:34:21 GMT" (always UTC). Called with every API response,
# but only syncs at startup and then every CLOCK_RESYNC_SECONDS.
def sync_clock(headers):
    global clock_synced, last_clock_sync
    if clock_synced and time.monotonic()-last_clock_sync<CLOCK_RESYNC_SECONDS:
        return
    date=""
    for name,value in headers.items():
        if name.lower()=="date":
            date=value
    try:
        _,day,month,year,hms,_=date.split()
        hour,minute,second=hms.split(":")
        utc=time.mktime((int(year),MONTHS.index(month)+1,int(day),int(hour),int(minute),int(second),0,-1,-1))
    except ValueError:
        print("Clock sync failed, unexpected Date header: "+repr(date))
        return
    rtc.RTC().datetime=time.localtime(utc+utc_offset(utc))
    clock_synced=True
    last_clock_sync=time.monotonic()
    print("Clock synced")

def update_clock():
    if clock_synced:
        now=time.localtime()
        text="%02d:%02d" % (now.tm_hour, now.tm_min)
    else:
        text="--:--"
    if clock_label.text!=text:
        clock_label.text=text

# What to show when there are no flights: the clock if it's on, else nothing
def show_idle_screen():
    if clock_on:
        update_clock()
        display.root_group = clock_group
    else:
        clear_flight()
        display.root_group = g

# Set when the search range changes, so the main loop checks for flights
# straight away instead of waiting for the next QUERY_DELAY
check_now=False
# When UP was last pressed (supervisor.ticks_ms), while waiting to see if it's
# a double press; None when not waiting
up_pressed_at=None

# Milliseconds from tick count a to b (the tick counter wraps round every ~6 days)
def ticks_diff(b, a):
    return ((b-a+(1<<28)) & ((1<<29)-1)) - (1<<28)

def toggle_clock():
    global clock_on
    clock_on=not clock_on
    print("Clock "+("on" if clock_on else "off"))
    save_settings()
    if idle:
        show_idle_screen()

def next_range():
    global range_index, check_now
    range_index=(range_index+1)%len(RANGE_OPTIONS_NM)
    print("Range "+str(RANGE_OPTIONS_NM[range_index])+"nm")
    save_settings()
    check_now=True
    show_message("Range "+str(RANGE_OPTIONS_NM[range_index])+"nm")

# Show a message in the middle of the panel for MESSAGE_SECONDS, then put back
# whatever was showing before
def show_message(text):
    previous=display.root_group
    message_label.text=text
    display.root_group = message_group
    for i in range(int(MESSAGE_SECONDS*10)):
        time.sleep(0.1)
        w.feed()
    display.root_group = previous

# UP: single press toggles the clock, double press cycles the brightness.
# DOWN: cycles the search range.
# Uses each press's own timestamp, so a double press still counts if the board
# was busy (e.g. waiting for a network request) while it happened.
def handle_buttons():
    global up_pressed_at
    event=buttons.events.get()
    while event:
        if event.pressed:
            if event.key_number==BUTTON_UP:
                if up_pressed_at is not None and ticks_diff(event.timestamp, up_pressed_at)<=DOUBLE_PRESS_MS:
                    up_pressed_at=None
                    next_brightness()
                else:
                    if up_pressed_at is not None:
                        # the earlier press was a single press we hadn't acted on yet
                        toggle_clock()
                    up_pressed_at=event.timestamp
            elif event.key_number==BUTTON_DOWN:
                next_range()
        event=buttons.events.get()
    # no second press in time, so it was a single press
    if up_pressed_at is not None and ticks_diff(supervisor.ticks_ms(), up_pressed_at)>DOUBLE_PRESS_MS:
        up_pressed_at=None
        toggle_clock()

# Call regularly from anything that runs for a while: keeps the watchdog happy
# and responds to button presses
def service():
    w.feed()
    handle_buttons()


# --- Networking ---

# Reconnect to WiFi if the connection has dropped (up to 10 attempts)
def checkConnection():
    print("Check and reconnect WiFi")
    attempts=10
    attempt=1
    while esp.status != adafruit_esp32spi.WL_CONNECTED and attempt<attempts:
        print("Connect attempt "+str(attempt)+" of "+str(attempts))
        w.feed()
        wifi.reset()
        w.feed()
        try:
            wifi.connect()
        except OSError as e:
            print("WiFi connect failed: "+repr(e))
        attempt+=1
    if esp.status == adafruit_esp32spi.WL_CONNECTED:
        print("Successfully connected.")
    else:
        print("Failed to connect.")

# GET a URL (or POST, if body is given) and return the parsed JSON. Retries once,
# reconnecting WiFi if needed. Returns None if both attempts fail.
def fetch_json(url, body=None):
    print("DEBUG lookup URL: "+url)
    for attempt in (1,2):
        # give the AirLift a moment to fully close the previous connection
        # before opening a new one
        time.sleep(0.75)
        try:
            if body is None:
                raw=requests.get(url=url,headers=rheaders)
            else:
                raw=requests.post(url=url,json=body,headers=rheaders)
            with raw:
                print("DEBUG lookup HTTP status: "+str(raw.status_code))
                sync_clock(raw.headers)
                return raw.json()
        except NETWORK_ERRORS as e:
            print("DEBUG lookup attempt "+str(attempt)+" failed: "+repr(e))
            w.feed()
            checkConnection()
    return None

def read_text(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return ""

# If GitHub has a release that isn't installed (or skipped after crashing),
# reset the board so the updater installs it. Downloading is left to the
# updater, which runs before the display has used up the memory.
def check_for_update():
    if not UPDATE_MANIFEST_URL:
        return
    manifest=fetch_json(UPDATE_MANIFEST_URL)
    version=manifest.get("version") if isinstance(manifest, dict) else None
    if not version or version in (read_text("/version.txt"), read_text("/skip_version.txt")):
        return
    print("New version "+version+" available, resetting to install it")
    show_message("Updating")
    microcontroller.reset()


# --- Flight data ---

# Great-circle distance in km between two points
def distance_km(lat1, lon1, lat2, lon2):
    lat1,lon1,lat2,lon2=map(math.radians,(lat1,lon1,lat2,lon2))
    a=math.sin((lat2-lat1)/2)**2+math.cos(lat1)*math.cos(lat2)*math.sin((lon2-lon1)/2)**2
    return 12742*math.asin(math.sqrt(a))

# Compass bearing in degrees from the first point to the second
def bearing_deg(lat1, lon1, lat2, lon2):
    lat1,lon1,lat2,lon2=map(math.radians,(lat1,lon1,lat2,lon2))
    y=math.sin(lon2-lon1)*math.cos(lat2)
    x=math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(lon2-lon1)
    return math.degrees(math.atan2(y,x))%360

# False if the plane clearly isn't flying origin -> dest: it's cruising (well
# away from both airports) but heading away from the destination.
def route_is_sane(lat, lon, track, origin, dest):
    if track is None:
        return True
    if distance_km(lat,lon,origin["lat"],origin["lon"])<ROUTE_CHECK_MIN_KM:
        return True
    if distance_km(lat,lon,dest["lat"],dest["lon"])<ROUTE_CHECK_MIN_KM:
        return True
    off=abs((bearing_deg(lat,lon,dest["lat"],dest["lon"])-track+180)%360-180)
    print("DEBUG heading is "+str(round(off))+" degrees off the destination")
    return off<=ROUTE_MAX_HEADING_OFF

# Short display name for an airport, e.g. "London Heathrow Airport" -> "London Heathrow"
def airport_name(airport):
    name=airport.get("name") or airport.get("location") or ""
    for word in (" International"," Airport"):
        name=name.replace(word,"")
    return name.strip()

# 3-letter IATA code (e.g. LHR), or the 4-letter ICAO code if there isn't one
def airport_code(airport):
    return airport.get("iata") or airport.get("icao") or "?"

# Ask adsb.im for this flight's route, sending the plane's live position so it
# can check the route is plausible. Returns (origin, destination) airport dicts,
# or None if there's no route we trust.
def get_route(ac, callsign):
    lat,lon=ac.get("lat"),ac.get("lon")
    if lat is None or lon is None:
        return None
    response=fetch_json(ROUTE_LOOKUP_URL, {"planes":[{"callsign":callsign,"lat":lat,"lng":lon}]})
    if not (isinstance(response, list) and response and isinstance(response[0], dict)):
        print("DEBUG no route for "+callsign)
        return None
    route=response[0]
    print("DEBUG route "+str(route.get("_airport_codes_iata"))+" plausible="+str(route.get("plausible")))
    # only airports with a known position, as the checks below need it
    airports=[a for a in (route.get("_airports") or [])
              if isinstance(a, dict) and a.get("lat") is not None and a.get("lon") is not None]
    if not route.get("plausible") or len(airports)<2:
        return None

    # Multi-stop routes (e.g. A-B-C): pick the leg the plane is on - the one it's
    # the smallest detour from
    best=None
    for a,b in zip(airports,airports[1:]):
        detour=(distance_km(lat,lon,a["lat"],a["lon"])+distance_km(lat,lon,b["lat"],b["lon"])
                -distance_km(a["lat"],a["lon"],b["lat"],b["lon"]))
        if best is None or detour<best[0]:
            best=(detour,a,b)
    origin,dest=best[1],best[2]

    if not route_is_sane(lat, lon, ac.get("track"), origin, dest):
        print("DEBUG route rejected: plane isn't heading for "+airport_code(dest))
        return None
    return origin,dest

# (name, IATA code, colour) for an airline's ICAO code, e.g. "RYR" ->
# ("Ryanair","FR",0xF1C933). From AIRLINE_NAMES, else adsbdb (colour None).
# Returns ("","",None) if the airline is unknown.
def get_airline(icao):
    if icao in AIRLINE_NAMES:
        return AIRLINE_NAMES[icao]
    if icao not in airline_cache:
        airline_cache[icao]=("","",None)
        response=fetch_json(AIRLINE_LOOKUP_HEAD+icao)
        data=response.get("response") if isinstance(response, dict) else None
        if isinstance(data, list) and data and isinstance(data[0], dict):
            airline_cache[icao]=(data[0].get("name") or "", data[0].get("iata") or "", None)
        else:
            # adsbdb returns a plain string (e.g. "unknown airline") when it has no match
            print("DEBUG adsbdb has no airline "+icao+": "+str(data))
    return airline_cache[icao]

# Aircraft model in normal capitalisation, e.g. "BEAGLE Pup" -> "Beagle Pup".
# Codes like "DA-20" are left alone.
def friendly_model(desc):
    words=[]
    for word in desc.split():
        if word.isalpha() and word.isupper() and len(word)>3:
            word=word[0]+word[1:].lower()
        words.append(word)
    return " ".join(words)

# The model, shortened to fit the panel without scrolling: the full model if it
# fits, else without the manufacturer ("DA-20"), else the type code ("DV20")
def short_model(model, code):
    for text in (model, " ".join(model.split()[1:]), code):
        if text and len(text)<=MAX_STATIC_CHARS:
            return text
    return code or model[:MAX_STATIC_CHARS]

# Altitude with thousands separators, e.g. 1100 -> "1,100ft"
def altitude_text(alt):
    if alt is None:
        return "?"
    if alt=="ground":
        return "GND"
    digits=str(round(alt))
    groups=[]
    while len(digits)>3:
        groups.insert(0,digits[-3:])
        digits=digits[:-3]
    groups.insert(0,digits)
    return ",".join(groups)+"ft"

# Work out what to show for an aircraft from adsb.fi. Returns a dict for
# display_flight():
#   "text":         (short, long) text for each line - short is shown still,
#                   long is scrolled (skipped if empty or the same as short)
#   "route":        (origin code, dest code, origin name, dest name), drawn on
#                   line 1 in two colours instead of its text; or None
#   "line2_colour": colour for line 2 and the plane icon
#   "two_line":     True for the two-line private/light aircraft layout
def parse_flight(ac):
    callsign=(ac.get("flight") or "").strip()
    if "@" in callsign:
        # "@@@@@@@@" means the transponder has no callsign set
        callsign=""
    registration=ac.get("r") or ""
    aircraft_code=ac.get("t") or ""
    aircraft_desc=ac.get("desc") or ""
    print("Flight is called "+(callsign or registration or "(unknown)"))
    print("DEBUG aircraft type: "+repr(aircraft_code)+" "+repr(aircraft_desc)+", registration "+repr(registration))

    # Airline callsigns are 3 letters then a digit (BAW615). Private aircraft
    # often use their registration instead (GEJCK), which has no route or airline.
    is_airline=callsign[:3].isalpha() and callsign[3:4].isdigit()
    route=get_route(ac, callsign) if (is_airline and LOOKUP_ROUTES) else None
    w.feed()
    airline_name,airline_iata,airline_colour=get_airline(callsign[:3]) if is_airline else ("","",None)
    w.feed()

    # Private/light aircraft: two lines, the model and altitude. This includes
    # charter/business operators (e.g. GlobeAir) - they have a name but no IATA
    # code or route, so the airline layout would just repeat the callsign.
    if not (airline_name and (airline_iata or route)):
        print("DEBUG no airline, showing model and altitude only")
        model=friendly_model(aircraft_desc) or aircraft_code or "Aircraft"
        altitude=altitude_text(ac.get("alt_baro"))
        return {
            "text": ((short_model(model, aircraft_code), model), (altitude, altitude), ("","")),
            "route": None,
            "line2_colour": GA_ALTITUDE_COLOUR,
            "two_line": True,
        }

    # Airline flight: three lines.
    # Line 1: origin > destination codes, scrolling the full airport names.
    # Without a route we trust, the callsign, scrolling the registration.
    if route:
        origin,dest=route
        route_text=(airport_code(origin),airport_code(dest),airport_name(origin),airport_name(dest))
        line1=("","")
        print("DEBUG using route: "+route_text[2]+" > "+route_text[3])
    else:
        route_text=None
        line1=(callsign, registration or callsign)
        print("DEBUG no route available, line 1 shows callsign/registration")

    # Line 2: flight number, scrolling the airline name. The number is the
    # callsign with the IATA code in place of the ICAO one (RYR3N -> FR3N).
    flight_number=(airline_iata+callsign[3:]) if airline_iata else callsign
    print("DEBUG using airline: "+airline_name+" flight "+flight_number)

    # Line 3: aircraft type code, scrolling the full make and model. Aircraft
    # new to the databases have no type yet, so show the altitude instead.
    if aircraft_code:
        line3=(aircraft_code, aircraft_desc or aircraft_code)
    else:
        altitude=altitude_text(ac.get("alt_baro"))
        line3=(altitude, altitude)
    return {
        "text": (line1, (flight_number, airline_name), line3),
        "route": route_text,
        "line2_colour": airline_colour or ROW_TWO_COLOUR,
        "two_line": False,
    }

# The closest airborne aircraft within the search range (by adsb.fi's "dst"
# distance field), ignoring ones on the ground at airports. None if there are
# none or the request failed.
def get_flights():
    response=fetch_json(FLIGHT_SEARCH_HEAD+str(RANGE_OPTIONS_NM[range_index]))
    aircraft_list=response.get("ac") if isinstance(response, dict) else None
    if not aircraft_list:
        return None

    closest=None
    for ac in aircraft_list:
        if not isinstance(ac, dict) or ac.get("alt_baro")=="ground":
            continue
        if closest is None or ac.get("dst",9999) < closest.get("dst",9999):
            closest=ac
    return closest


# --- Main loop: find the closest aircraft, show it if it's new, wait, repeat ---

checkConnection()

last_hex=''  # the aircraft currently shown (its unique transponder "hex" code)
healthy=False  # set after the first time round the loop, see mark_healthy()
last_update_check=time.monotonic()  # the updater has just checked
while True:
    service()
    check_now=False
    try:
        ac=get_flights()
        service()

        if not ac:
            # forget the last flight, so it's shown again if it comes back
            last_hex=''
            idle=True
            show_idle_screen()
        elif ac.get("hex","")==last_hex:
            print("Same flight found, so keep showing it")
        else:
            last_hex=ac.get("hex","")
            idle=False
            print("New flight "+last_hex+" found, clear display")
            clear_flight()
            gc.collect()
            flight=parse_flight(ac)
            gc.collect()
            plane_animation(flight["line2_colour"])
            display_flight(flight)
    except Exception as e:
        # Unexpected data from one of the APIs shouldn't stop the display for
        # good - log it, show the idle screen and try again next time round
        print("Error showing flight: "+repr(e))
        idle=True
        show_idle_screen()

    if not healthy:
        mark_healthy()
        healthy=True
    if time.monotonic()-last_update_check>=UPDATE_CHECK_SECONDS:
        last_update_check=time.monotonic()
        try:
            check_for_update()
        except Exception as e:
            print("Update check failed: "+repr(e))

    # Wait until the next check, in short steps so buttons respond quickly
    for i in range(QUERY_DELAY*10):
        time.sleep(0.1)
        service()
        if check_now:
            break  # the search range changed - check again straight away
        if idle and clock_on:
            update_clock()
    gc.collect()
