from machine import I2C, Pin
import framebuf
import gc
import math
import network
import socket
import ssl
import time
import ujson


WIFI_SSID = "YOUR_WIFI_SSID"
WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"

OLED_WIDTH = 128
OLED_HEIGHT = 64
OLED_ADDR = 0x3C
OLED_SDA = 4
OLED_SCL = 15
OLED_RST = 16
ROTATE_90 = True
PROGRAM_BUTTON = 0

RADAR_CX = 32
RADAR_CY = 52
RADAR_R = 27
RANGE_KM = 15
QUERY_RADIUS_NM = 8
FETCH_SECONDS = 30
MAX_TARGETS = 20
HTTP_TIMEOUT_SECONDS = 8
FIRST_FETCH_DELAY_SECONDS = 5
WIFI_TIMEOUT_SECONDS = 12
FETCH_RETRY_SECONDS = 15
BUTTON_DEBOUNCE_MS = 50
BUTTON_LONG_MS = 900

HOME_LAT = 51.500000
HOME_LON = -0.120000
HOME_PLACE = "HOME"


class SSD1306:
    def __init__(self, width, height, i2c, addr=0x3C, rotate_90=False):
        self.phys_width = width
        self.phys_height = height
        self.rotate_90 = rotate_90
        self.width = height if rotate_90 else width
        self.height = width if rotate_90 else height
        self.i2c = i2c
        self.addr = addr
        self.pages = self.phys_height // 8
        self.buffer = bytearray(self.pages * self.phys_width)
        self.fb = framebuf.FrameBuffer(
            self.buffer, self.phys_width, self.phys_height, framebuf.MONO_VLSB
        )
        self.init_display()

    def cmd(self, cmd):
        self.i2c.writeto(self.addr, bytearray([0x80, cmd]))

    def data(self, buf):
        self.i2c.writeto(self.addr, b"\x40" + buf)

    def init_display(self):
        for cmd in (
            0xAE, 0x20, 0x00, 0x40, 0xA1, 0xA8, 0x3F, 0xC8,
            0xD3, 0x00, 0xDA, 0x12, 0xD5, 0x80, 0xD9, 0xF1,
            0xDB, 0x30, 0x81, 0xFF, 0xA4, 0xA6, 0x8D, 0x14, 0xAF
        ):
            self.cmd(cmd)
        self.fill(0)
        self.show()

    def fill(self, color):
        self.fb.fill(color)

    def text(self, text, x, y, color=1):
        if not self.rotate_90:
            self.fb.text(text, x, y, color)
            return
        text = str(text)
        if not text:
            return
        w = len(text) * 8
        buf = bytearray(w)
        temp = framebuf.FrameBuffer(buf, w, 8, framebuf.MONO_VLSB)
        temp.fill(0)
        temp.text(text, 0, 0, 1)
        for ty in range(8):
            for tx in range(w):
                if temp.pixel(tx, ty):
                    self.pixel(x + tx, y + ty, color)

    def pixel(self, x, y, color):
        if 0 <= x < self.width and 0 <= y < self.height:
            if self.rotate_90:
                self.fb.pixel(self.phys_width - 1 - y, x, color)
            else:
                self.fb.pixel(x, y, color)

    def line(self, x1, y1, x2, y2, color):
        dx = abs(x2 - x1)
        dy = -abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx + dy
        while True:
            self.pixel(x1, y1, color)
            if x1 == x2 and y1 == y2:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x1 += sx
            if e2 <= dx:
                err += dx
                y1 += sy

    def hline(self, x, y, w, color):
        self.line(x, y, x + w - 1, y, color)

    def vline(self, x, y, h, color):
        self.line(x, y, x, y + h - 1, color)

    def rect(self, x, y, w, h, color):
        self.hline(x, y, w, color)
        self.hline(x, y + h - 1, w, color)
        self.vline(x, y, h, color)
        self.vline(x + w - 1, y, h, color)

    def circle(self, cx, cy, r, color):
        x = r
        y = 0
        err = 0
        while x >= y:
            for px, py in (
                (cx + x, cy + y), (cx + y, cy + x), (cx - y, cy + x),
                (cx - x, cy + y), (cx - x, cy - y), (cx - y, cy - x),
                (cx + y, cy - x), (cx + x, cy - y)
            ):
                self.pixel(px, py, color)
            y += 1
            if err <= 0:
                err += 2 * y + 1
            if err > 0:
                x -= 1
                err -= 2 * x + 1

    def show(self):
        self.cmd(0x21)
        self.cmd(0)
        self.cmd(self.phys_width - 1)
        self.cmd(0x22)
        self.cmd(0)
        self.cmd(self.pages - 1)
        self.data(self.buffer)


def screen_message(lines):
    display.fill(0)
    for i, line in enumerate(lines[:6]):
        display.text(str(line)[:8], 0, i * 10, 1)
    display.show()


def reset_oled():
    rst = Pin(OLED_RST, Pin.OUT)
    rst.value(0)
    time.sleep_ms(50)
    rst.value(1)
    time.sleep_ms(50)


def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("wifi connecting")
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        started = time.ticks_ms()
        while (
            not wlan.isconnected()
            and time.ticks_diff(time.ticks_ms(), started) < WIFI_TIMEOUT_SECONDS * 1000
        ):
            time.sleep_ms(350)
    if wlan.isconnected():
        print("wifi ok", wlan.ifconfig()[0], wlan.status())
        return wlan
    print("wifi failed", wlan.status())
    return wlan


def fetch_json(url):
    gc.collect()
    try:
        return ujson.loads(http_get(url))
    finally:
        gc.collect()


def split_url(url):
    proto, rest = url.split("://", 1)
    if "/" in rest:
        host, path = rest.split("/", 1)
        path = "/" + path
    else:
        host = rest
        path = "/"
    return proto, host, path


def http_get(url):
    proto, host, path = split_url(url)
    port = 443 if proto == "https" else 80
    addr = socket.getaddrinfo(host, port)[0][-1]
    s = socket.socket()
    s.settimeout(HTTP_TIMEOUT_SECONDS)
    try:
        s.connect(addr)
        if proto == "https":
            s = ssl.wrap_socket(s, server_hostname=host)
            time.sleep_ms(50)
        request = (
            "GET %s HTTP/1.1\r\n"
            "Host: %s\r\n"
            "User-Agent: heltec-oled-radar\r\n"
            "Connection: close\r\n\r\n"
        ) % (path, host)
        s.write(request.encode())
        time.sleep_ms(50)
        chunks = []
        reads = 0
        while True:
            try:
                chunk = s.read(512)
            except OSError:
                if chunks:
                    break
                raise
            if not chunk:
                break
            chunks.append(chunk)
            reads += 1
            if reads % 4 == 0:
                time.sleep_ms(1)
        raw = b"".join(chunks)
    finally:
        s.close()

    head, body = raw.split(b"\r\n\r\n", 1)
    status = head.split(b" ", 2)[1]
    if status != b"200":
        raise OSError("http " + status.decode())
    if b"Transfer-Encoding: chunked" in head or b"transfer-encoding: chunked" in head:
        body = decode_chunked(body)
    return body.decode()


def decode_chunked(data):
    out = []
    pos = 0
    while True:
        end = data.find(b"\r\n", pos)
        if end < 0:
            break
        size = int(data[pos:end], 16)
        if size == 0:
            break
        start = end + 2
        out.append(data[start:start + size])
        pos = start + size + 2
    return b"".join(out)


def locate():
    return HOME_LAT, HOME_LON, HOME_PLACE


def flight_url(lat, lon):
    return "https://api.adsb.lol/v2/point/%.6f/%.6f/%d" % (
        lat,
        lon,
        QUERY_RADIUS_NM,
    )


def km_from_center(lat0, lon0, lat, lon):
    dy = (lat - lat0) * 111.32
    dx = (lon - lon0) * 111.32 * math.cos(math.radians(lat0))
    return dx, dy, math.sqrt(dx * dx + dy * dy)


def fetch_targets(lat, lon):
    try:
        print("fetch flights")
        data = fetch_json(flight_url(lat, lon))
    except Exception as err:
        print("fetch failed", repr(err))
        return None
    aircraft = data.get("ac") or []
    targets = []
    for aircraft_item in aircraft:
        if aircraft_item.get("lat") is None or aircraft_item.get("lon") is None:
            continue
        dx, dy, dist = km_from_center(
            lat,
            lon,
            float(aircraft_item.get("lat")),
            float(aircraft_item.get("lon")),
        )
        if dist > RANGE_KM:
            continue
        name = (aircraft_item.get("flight") or aircraft_item.get("hex") or "").strip()
        alt = aircraft_item.get("alt_baro")
        if not isinstance(alt, int):
            alt = aircraft_item.get("alt_geom")
        if not isinstance(alt, int):
            alt = 0
        heading = aircraft_item.get("track")
        if heading is None:
            heading = 0
        seen = aircraft_item.get("seen_pos")
        if seen is None:
            seen = aircraft_item.get("seen") or 0
        speed = aircraft_item.get("gs")
        if speed is None:
            speed = 0
        aircraft_type = aircraft_item.get("t") or aircraft_item.get("hex") or ""
        targets.append(
            (
                dist,
                dx,
                dy,
                name[:7],
                int(alt),
                int(heading),
                int(speed),
                aircraft_type[:7],
                int(seen),
                time.time(),
            )
        )
    targets.sort(key=lambda item: item[0])
    print("targets", len(targets))
    return targets[:MAX_TARGETS]


def plot_target(target):
    _, dx, dy, _, _, heading, _, _, seen, fetched_at = target
    dx, dy = projected_offset(dx, dy, heading, target[6], seen, fetched_at)
    x = int(RADAR_CX + dx / RANGE_KM * RADAR_R)
    y = int(RADAR_CY - dy / RANGE_KM * RADAR_R)
    display.pixel(x, y, 1)
    display.pixel(x + 1, y, 1)
    display.pixel(x, y + 1, 1)
    nose = math.radians(heading - 90)
    display.pixel(x + int(math.cos(nose) * 3), y + int(math.sin(nose) * 3), 1)


def projected_offset(dx, dy, heading, speed_kt, seen_seconds, fetched_at):
    age = max(0, time.time() - fetched_at + seen_seconds)
    km = speed_kt * 1.852 * age / 3600
    angle = math.radians(heading)
    return dx + math.sin(angle) * km, dy + math.cos(angle) * km


def compass_from_offset(dx, dy):
    angle = (math.degrees(math.atan2(dx, dy)) + 360) % 360
    dirs = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")
    return dirs[int((angle + 22.5) // 45) % 8]


def compass_from_degrees(angle):
    dirs = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")
    return dirs[int((angle + 22.5) // 45) % 8]


def draw_radar(angle, targets, place, stale=False, wifi_ok=True):
    display.fill(0)
    display.text(place.replace(" ", "")[:6], 0, 0, 1)
    status = "%02d" % len(targets) if wifi_ok else "WF"
    display.text(status, display.width - 16, 0, 1)
    display.circle(RADAR_CX, RADAR_CY, RADAR_R, 1)
    display.circle(RADAR_CX, RADAR_CY, RADAR_R // 2, 1)
    display.hline(RADAR_CX - RADAR_R, RADAR_CY, RADAR_R * 2 + 1, 1)
    display.vline(RADAR_CX, RADAR_CY - RADAR_R, RADAR_R * 2 + 1, 1)

    sweep = math.radians(angle)
    sx = int(RADAR_CX + math.cos(sweep) * RADAR_R)
    sy = int(RADAR_CY + math.sin(sweep) * RADAR_R)
    display.line(RADAR_CX, RADAR_CY, sx, sy, 1)

    for target in targets:
        plot_target(target)

    if targets:
        nearest = targets[0]
        distance, dx, dy, callsign, altitude, heading, speed, _, seen, fetched_at = nearest
        dx, dy = projected_offset(dx, dy, heading, speed, seen, fetched_at)
        distance = math.sqrt(dx * dx + dy * dy)
        callsign = (callsign or "ICAO")[:7]
        bearing = compass_from_offset(dx, dy)
        distance_line = "%02dkm %s" % (int(distance), bearing)
        altitude_line = "%dft" % altitude
        display.text(callsign[:8], 0, 98, 1)
        display.text(distance_line[:8], 0, 108, 1)
        display.text(altitude_line[:8], 0, 118, 1)
    elif stale:
        display.text("NODATA", 8, 114, 1)
    else:
        display.text("SCAN", 16, 114, 1)
    display.show()


def draw_detail(target, index, total):
    distance, dx, dy, callsign, altitude, heading, speed, aircraft_type, seen, fetched_at = target
    dx, dy = projected_offset(dx, dy, heading, speed, seen, fetched_at)
    distance = math.sqrt(dx * dx + dy * dy)
    display.fill(0)
    display.text("%02d/%02d" % (index + 1, total), 16, 0, 1)
    display.text((callsign or "ICAO")[:8], 0, 18, 1)
    display.text(aircraft_type[:8], 0, 30, 1)
    display.text(("%02dkm %s" % (int(distance), compass_from_offset(dx, dy)))[:8], 0, 48, 1)
    display.text(("%dft" % altitude)[:8], 0, 60, 1)
    display.text(("H%s%03d" % (compass_from_degrees(heading), heading))[:8], 0, 78, 1)
    display.text(("%dkt" % speed)[:8], 0, 90, 1)
    display.text("HOLD=RAD", 0, 116, 1)
    display.show()


def read_button_event(button, state):
    now = time.ticks_ms()
    pressed = button.value() == 0
    event = None

    if pressed != state["raw"]:
        state["raw"] = pressed
        state["changed"] = now

    if time.ticks_diff(now, state["changed"]) > BUTTON_DEBOUNCE_MS:
        if pressed != state["stable"]:
            state["stable"] = pressed
            if pressed:
                state["pressed_at"] = now
                state["long_sent"] = False
            else:
                if not state["long_sent"]:
                    event = "short"

    if (
        state["stable"]
        and not state["long_sent"]
        and time.ticks_diff(now, state["pressed_at"]) > BUTTON_LONG_MS
    ):
        state["long_sent"] = True
        event = "long"

    return event


lat, lon, place = locate()
targets = []
last_fetch = time.time() - FETCH_SECONDS + FIRST_FETCH_DELAY_SECONDS
stale = False
angle = 0
detail_mode = False
selected_target = 0

reset_oled()
i2c = I2C(0, scl=Pin(OLED_SCL), sda=Pin(OLED_SDA), freq=400000)
display = SSD1306(OLED_WIDTH, OLED_HEIGHT, i2c, OLED_ADDR, ROTATE_90)
button = Pin(PROGRAM_BUTTON, Pin.IN, Pin.PULL_UP)
button_state = {
    "raw": False,
    "stable": False,
    "changed": time.ticks_ms(),
    "pressed_at": 0,
    "long_sent": False,
}
draw_radar(angle, targets, place, stale, False)
wlan = connect_wifi()
draw_radar(angle, targets, place, stale, wlan.isconnected())

while True:
    button_event = read_button_event(button, button_state)
    if button_event == "short" and targets:
        if not detail_mode:
            selected_target = 0
        else:
            selected_target = (selected_target + 1) % len(targets)
        detail_mode = True
    elif button_event == "long":
        detail_mode = False

    if not wlan.isconnected():
        wlan = connect_wifi()

    if not detail_mode and time.time() - last_fetch > FETCH_SECONDS:
        draw_radar(angle, targets, place, stale, wlan.isconnected())
        found = fetch_targets(lat, lon) if wlan.isconnected() else None
        last_fetch = time.time()
        if found is None:
            stale = True
        elif found:
            targets = found
            if selected_target >= len(targets):
                selected_target = 0
            stale = False
        else:
            stale = True

    if detail_mode and targets:
        draw_detail(targets[selected_target], selected_target, len(targets))
    else:
        detail_mode = False
        draw_radar(angle, targets, place, stale, wlan.isconnected())
        angle = (angle + 12) % 360
    time.sleep_ms(120)
