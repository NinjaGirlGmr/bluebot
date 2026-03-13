#!/usr/bin/env python3
import fcntl
import os
import subprocess
import time

import Jetson.GPIO as GPIO
from PIL import Image, ImageDraw, ImageFont

# ===== CONFIG =====
GPIO_PIN = 7  # BOARD numbering: physical pin 7
HOTSPOT_CONN = "BluebotHotspot"
CLIENT_CONN = "Bazinga5"
POLL_SEC = 0.2
DEBOUNCE_SEC = 0.05
NMCLI_READY_TIMEOUT_SEC = 45.0
NMCLI_READY_POLL_SEC = 1.0
MODE_RECONCILE_SEC = 10.0
DISPLAY_REFRESH_SEC = 2.0

# 0.96" I2C OLED (typically SSD1306, 128x64, address 0x3C)
OLED_ENABLED = True
OLED_I2C_BUS = None  # Auto-detect when None
OLED_I2C_BUS_CANDIDATES = (1, 0, 7)
OLED_I2C_ADDR = 0x3C
OLED_WIDTH = 128
OLED_HEIGHT = 64
OLED_BUS_WAIT_SEC = 8.0
# ==================

I2C_SLAVE = 0x0703


class OledStatusDisplay:
    def __init__(self):
        self.enabled = False
        self.fd = None
        self.last_lines = ()
        self.pages = OLED_HEIGHT // 8
        self.font = ImageFont.load_default()

        if not OLED_ENABLED:
            return

        bus = self._resolve_bus()
        if bus is None:
            print("OLED disabled: no /dev/i2c-* device found.")
            return

        dev_path = f"/dev/i2c-{bus}"
        try:
            self.fd = os.open(dev_path, os.O_RDWR)
            fcntl.ioctl(self.fd, I2C_SLAVE, OLED_I2C_ADDR)
            self.enabled = True
            self._initialize()
            self.show_lines(["Bluebot WiFi", "OLED online"])
            print(f"OLED ready on {dev_path} (0x{OLED_I2C_ADDR:02X}).")
        except OSError as exc:
            print(f"OLED disabled: failed to open {dev_path}: {exc}")
            self.close()

    def _resolve_bus(self):
        buses = (OLED_I2C_BUS,) if OLED_I2C_BUS is not None else OLED_I2C_BUS_CANDIDATES
        deadline = time.monotonic() + OLED_BUS_WAIT_SEC
        while True:
            for bus in buses:
                if os.path.exists(f"/dev/i2c-{bus}"):
                    return bus
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.25)

    def _write(self, control, payload):
        if not self.enabled or self.fd is None or not payload:
            return
        try:
            # Keep transfers modest for broad controller compatibility.
            for idx in range(0, len(payload), 16):
                chunk = payload[idx:idx + 16]
                os.write(self.fd, bytes([control]) + chunk)
        except OSError as exc:
            print(f"OLED write failed: {exc}")
            self.close()

    def _command(self, *cmds):
        self._write(0x00, bytes(cmds))

    def _data(self, data):
        self._write(0x40, data)

    def _initialize(self):
        self._command(
            0xAE,       # Display OFF
            0xD5, 0x80, # Clock divide
            0xA8, 0x3F, # Multiplex ratio (64)
            0xD3, 0x00, # Display offset
            0x40,       # Start line
            0x8D, 0x14, # Charge pump ON
            0x20, 0x00, # Horizontal addressing mode
            0xA1,       # Segment remap
            0xC8,       # COM scan dec
            0xDA, 0x12, # COM pins
            0x81, 0xCF, # Contrast
            0xD9, 0xF1, # Pre-charge
            0xDB, 0x40, # VCOM detect
            0xA4,       # Resume RAM content display
            0xA6,       # Normal display
            0xAF,       # Display ON
        )
        self.clear()

    def clear(self):
        if not self.enabled:
            return
        self._command(0x21, 0, OLED_WIDTH - 1, 0x22, 0, self.pages - 1)
        self._data(bytes([0x00]) * (OLED_WIDTH * self.pages))
        self.last_lines = ()

    def show_lines(self, lines):
        if not self.enabled:
            return

        normalized = tuple((line or "")[:24] for line in lines[:6])
        if normalized == self.last_lines:
            return

        image = Image.new("1", (OLED_WIDTH, OLED_HEIGHT))
        draw = ImageDraw.Draw(image)
        y = 0
        for line in normalized:
            draw.text((0, y), line, font=self.font, fill=255)
            y += 10

        frame = bytearray(OLED_WIDTH * self.pages)
        for page in range(self.pages):
            for x in range(OLED_WIDTH):
                value = 0
                for bit in range(8):
                    y = page * 8 + bit
                    if image.getpixel((x, y)):
                        value |= (1 << bit)
                frame[(page * OLED_WIDTH) + x] = value

        self._command(0x21, 0, OLED_WIDTH - 1, 0x22, 0, self.pages - 1)
        self._data(bytes(frame))
        self.last_lines = normalized

    def show_status(self, mode, state, detail=""):
        lines = ["Bluebot WiFi", f"Mode: {mode}", state]
        if detail:
            lines.append(detail)
        self.show_lines(lines)

    def close(self):
        if self.fd is not None:
            try:
                self._command(0xAE)  # Display OFF
            except OSError:
                pass
            try:
                os.close(self.fd)
            except OSError:
                pass
        self.fd = None
        self.enabled = False
        self.last_lines = ()


def run(args):
    try:
        return subprocess.run(args, check=False, capture_output=True, text=True)
    except FileNotFoundError as exc:
        return subprocess.CompletedProcess(args=args, returncode=127, stdout="", stderr=str(exc))


def wait_for_nmcli_ready(timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        r = run(["nmcli", "-t", "-f", "RUNNING", "general"])
        if r.returncode == 0 and r.stdout.strip().lower() == "running":
            return True
        time.sleep(NMCLI_READY_POLL_SEC)
    return False


def apply_desired_mode(mode: str):
    if mode == "hotspot":
        switch_to_hotspot()
    else:
        switch_to_client()


def connection_is_active(name: str) -> bool:
    r = run(["nmcli", "-t", "-f", "NAME", "connection", "show", "--active"])
    if r.returncode != 0:
        print("Failed to read active connections:")
        print(r.stderr.strip())
        return False
    active_names = {line.strip() for line in r.stdout.splitlines() if line.strip()}
    return name in active_names


def get_active_wifi_connection():
    r = run(["nmcli", "-t", "-f", "DEVICE,TYPE,STATE,CONNECTION", "device", "status"])
    if r.returncode != 0:
        return None, None

    for line in r.stdout.splitlines():
        parts = line.split(":", 3)
        if len(parts) != 4:
            continue
        device, dev_type, state, conn = [p.strip() for p in parts]
        if dev_type == "wifi" and conn and conn != "--" and state in ("connected", "connecting"):
            return conn, device
    return None, None


def get_connection_ssid(connection_name: str) -> str:
    if not connection_name:
        return "N/A"

    r = run(["nmcli", "-g", "802-11-wireless.ssid", "connection", "show", connection_name])
    if r.returncode == 0:
        ssid = r.stdout.strip()
        if ssid:
            return ssid
    return connection_name


def get_device_ipv4(device: str) -> str:
    if not device:
        return "N/A"

    r = run(["nmcli", "-g", "IP4.ADDRESS", "device", "show", device])
    if r.returncode != 0:
        return "N/A"

    for line in r.stdout.splitlines():
        entry = line.strip()
        if entry:
            return entry.split("/", 1)[0]
    return "N/A"


def mode_from_connection(connection_name: str) -> str:
    if connection_name == HOTSPOT_CONN:
        return "hotspot"
    if connection_name == CLIENT_CONN:
        return "client"
    return "unknown"


def update_oled_network_view(desired_mode: str, state: str = "Active"):
    target_conn = HOTSPOT_CONN if desired_mode == "hotspot" else CLIENT_CONN
    active_conn, active_device = get_active_wifi_connection()

    displayed_mode = mode_from_connection(active_conn)
    if displayed_mode == "unknown":
        displayed_mode = desired_mode

    if active_conn:
        ssid = get_connection_ssid(active_conn)
        ip_addr = get_device_ipv4(active_device)
    else:
        ssid = get_connection_ssid(target_conn)
        ip_addr = "N/A"

    lines = [
        "Bluebot WiFi",
        f"Mode: {displayed_mode}",
        f"SSID: {ssid}",
        f"IP: {ip_addr}",
    ]
    if state and state != "Active":
        lines.append(state)
    oled.show_lines(lines)


GPIO.setmode(GPIO.BOARD)
GPIO.setup(GPIO_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

last_state = GPIO.input(GPIO_PIN)
current_mode = None  # "hotspot" or "client"
oled = OledStatusDisplay()
oled.show_lines(["Bluebot WiFi", "Starting..."])


def switch_to_hotspot():
    global current_mode
    if current_mode == "hotspot" or connection_is_active(HOTSPOT_CONN):
        current_mode = "hotspot"
        update_oled_network_view("hotspot", "Active")
        return

    print("Switching to hotspot mode...")
    update_oled_network_view("hotspot", "Switching...")
    run(["nmcli", "connection", "down", CLIENT_CONN])
    r = run(["nmcli", "connection", "up", HOTSPOT_CONN])
    if r.returncode == 0:
        current_mode = "hotspot"
        print("Hotspot mode active.")
        update_oled_network_view("hotspot", "Active")
    else:
        print("Failed to enable hotspot:")
        print(r.stderr.strip())
        update_oled_network_view("hotspot", "FAILED")


def switch_to_client():
    global current_mode
    if current_mode == "client" or connection_is_active(CLIENT_CONN):
        current_mode = "client"
        update_oled_network_view("client", "Active")
        return

    print("Switching to client mode...")
    update_oled_network_view("client", "Switching...")
    run(["nmcli", "connection", "down", HOTSPOT_CONN])
    r = run(["nmcli", "connection", "up", CLIENT_CONN])
    if r.returncode == 0:
        current_mode = "client"
        print("Client mode active.")
        update_oled_network_view("client", "Active")
    else:
        print("Failed to enable client mode:")
        print(r.stderr.strip())
        update_oled_network_view("client", "FAILED")


try:
    desired_mode = "hotspot" if last_state == GPIO.LOW else "client"
    update_oled_network_view(desired_mode, "Waiting for NM...")
    if not wait_for_nmcli_ready(NMCLI_READY_TIMEOUT_SEC):
        print("NetworkManager not ready after timeout; continuing with periodic retries.")
        update_oled_network_view(desired_mode, "NM not ready")

    # Set initial state on startup.
    apply_desired_mode(desired_mode)
    last_reconcile = time.monotonic()
    last_display_refresh = last_reconcile

    while True:
        state = GPIO.input(GPIO_PIN)
        if state != last_state:
            time.sleep(DEBOUNCE_SEC)
            state2 = GPIO.input(GPIO_PIN)
            if state2 == state:
                last_state = state
                desired_mode = "hotspot" if state == GPIO.LOW else "client"
                apply_desired_mode(desired_mode)
                last_reconcile = time.monotonic()

        now = time.monotonic()
        if now - last_reconcile >= MODE_RECONCILE_SEC:
            apply_desired_mode(desired_mode)
            last_reconcile = now
        if now - last_display_refresh >= DISPLAY_REFRESH_SEC:
            update_oled_network_view(desired_mode, "Active")
            last_display_refresh = now
        time.sleep(POLL_SEC)

except KeyboardInterrupt:
    pass
finally:
    oled.close()
    GPIO.cleanup()
