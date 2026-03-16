#!/usr/bin/env python3
import fcntl
import os
import subprocess
import time
from typing import Optional, Set

from PIL import Image, ImageDraw, ImageFont

# ===== CONFIG =====
HOTSPOT_CONN = "BluebotHotspot"
CLIENT_CONN = "Bazinga5"
CLIENT_SSID = None  # Set to an SSID string to override auto-detection from CLIENT_CONN.
POLL_SEC = 0.2
NMCLI_READY_TIMEOUT_SEC = 45.0
NMCLI_READY_POLL_SEC = 1.0
MODE_RECONCILE_SEC = 10.0
DISPLAY_REFRESH_SEC = 2.0

# 0.96" I2C OLED (typically SSD1306, 128x64, address 0x3C)
OLED_ENABLED = True
OLED_I2C_BUS = 7
OLED_I2C_BUS_CANDIDATES = (4, 1, 0, 7)
OLED_I2C_ADDR = 0x3C
OLED_WIDTH = 128
OLED_HEIGHT = 64
OLED_BUS_WAIT_SEC = 8.0
OLED_TOP_BAND_PX = 16
OLED_TOP_FONT_SIZE = 12
OLED_BODY_FONT_SIZE = 9
OLED_BODY_LINE_STEP = 12
OLED_FONT_CANDIDATES = (
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
    "/usr/share/fonts/truetype/liberation2/LiberationMono-Regular.ttf",
)
# ==================

I2C_SLAVE = 0x0703
oled = None
current_mode = None
cpu_prev_total = None
cpu_prev_idle = None


class OledError(RuntimeError):
    pass


class OledStatusDisplay:
    def __init__(self):
        self.enabled = False
        self.fd = None
        self.last_lines = ()
        self.pages = OLED_HEIGHT // 8
        self.top_font = self._load_font(OLED_TOP_FONT_SIZE)
        self.body_font = self._load_font(OLED_BODY_FONT_SIZE)
        self.top_font_height = self._measure_text("Ag", self.top_font)[1]
        self.body_font_height = self._measure_text("Ag", self.body_font)[1]
        self.top_line_y = max(0, (OLED_TOP_BAND_PX - self.top_font_height) // 2)
        self.blue_line_y = self._blue_line_positions()
        self.max_blue_lines = len(self.blue_line_y)

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
            self.show_lines([format_usage_line(), "Mode: auto", "OLED online"])
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

    def _load_font(self, size):
        for path in OLED_FONT_CANDIDATES:
            try:
                return ImageFont.truetype(path, size)
            except Exception:
                continue
        return ImageFont.load_default()

    def _measure_text(self, text, font):
        if hasattr(font, "getsize"):
            return font.getsize(text)
        if hasattr(font, "getbbox"):
            left, top, right, bottom = font.getbbox(text)
            return (right - left, bottom - top)
        return (len(text) * 6, 8)

    def _clip_to_width(self, text, width, font):
        clipped = text or ""
        while clipped and self._measure_text(clipped, font)[0] > width:
            clipped = clipped[:-1]
        return clipped

    def _blue_line_positions(self):
        start_y = OLED_TOP_BAND_PX
        if OLED_HEIGHT == 64:
            return [16, 28, 40, 52]
        positions = []
        y = start_y
        while y + self.body_font_height <= OLED_HEIGHT and len(positions) < 4:
            positions.append(y)
            y += OLED_BODY_LINE_STEP
        return positions or [0]

    def _write(self, control, payload):
        if not self.enabled or self.fd is None or not payload:
            return
        try:
            # Keep transfers modest for broad controller compatibility.
            for idx in range(0, len(payload), 16):
                chunk = payload[idx:idx + 16]
                os.write(self.fd, bytes([control]) + chunk)
        except OSError as exc:
            # Disable OLED immediately on bus errors so we don't recurse through close().
            message = f"OLED write failed: {exc}"
            print(f"{message}; triggering fail-safe.")
            fd = self.fd
            self.fd = None
            self.enabled = False
            self.last_lines = ()
            if fd is not None:
                try:
                    os.close(fd)
                except OSError:
                    pass
            raise OledError(message) from exc

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

        top_line = self._clip_to_width((lines[0] if lines else ""), OLED_WIDTH, self.top_font)
        blue_lines = [
            self._clip_to_width((line or ""), OLED_WIDTH, self.body_font)
            for line in lines[1 : 1 + self.max_blue_lines]
        ]
        normalized = tuple([top_line] + blue_lines)
        if normalized == self.last_lines:
            return

        image = Image.new("1", (OLED_WIDTH, OLED_HEIGHT))
        draw = ImageDraw.Draw(image)
        draw.text((0, self.top_line_y), top_line, font=self.top_font, fill=255)
        for idx, line in enumerate(blue_lines):
            draw.text((0, self.blue_line_y[idx]), line, font=self.body_font, fill=255)

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
        lines = [format_usage_line(), mode_header_line(mode), state]
        if detail:
            lines.append(detail)
        self.show_lines(lines)

    def close(self):
        fd = self.fd
        was_enabled = self.enabled
        self.fd = None
        self.enabled = False
        self.last_lines = ()
        if fd is None:
            return
        if was_enabled:
            try:
                # Send "display off" directly to avoid calling _write() during shutdown.
                os.write(fd, bytes([0x00, 0xAE]))
            except OSError:
                pass
        try:
            os.close(fd)
        except OSError:
            pass


def run(args):
    try:
        return subprocess.run(args, check=False, capture_output=True, text=True)
    except FileNotFoundError as exc:
        return subprocess.CompletedProcess(args=args, returncode=127, stdout="", stderr=str(exc))


def command_error_text(result: subprocess.CompletedProcess) -> str:
    message = (result.stderr or result.stdout or "").strip()
    return message or f"exit code {result.returncode}"


def mode_header_line(mode: str) -> str:
    value = (mode or "unknown").strip() or "unknown"
    return f"Mode: {value}"


def bail_out_to_client(reason: str):
    print(f"FAILSAFE: {reason}")
    if oled is not None:
        try:
            oled.show_lines([format_usage_line(), mode_header_line("failsafe"), "Bringing up", CLIENT_CONN])
        except Exception as exc:
            print(f"FAILSAFE: OLED unavailable while handling failure: {exc}")

    # Best-effort cleanup; ignore "already down" and similar return codes here.
    run(["nmcli", "connection", "down", HOTSPOT_CONN])
    result = run(["nmcli", "connection", "up", CLIENT_CONN])
    if result.returncode == 0:
        print(f"FAILSAFE: {CLIENT_CONN} active.")
        if oled is not None:
            try:
                oled.show_lines([format_usage_line(), mode_header_line("failsafe"), CLIENT_CONN, "Active"])
            except Exception as exc:
                print(f"FAILSAFE: OLED unavailable while confirming {CLIENT_CONN}: {exc}")
    else:
        error_text = command_error_text(result)
        print(f"FAILSAFE: failed to bring up {CLIENT_CONN}: {error_text}")
        if oled is not None:
            try:
                oled.show_lines([format_usage_line(), mode_header_line("failsafe"), CLIENT_CONN, "FAILED"])
            except Exception as exc:
                print(f"FAILSAFE: OLED unavailable while reporting fallback failure: {exc}")


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
        raise RuntimeError(f"Failed to read active connections: {command_error_text(r)}")
    active_names = {line.strip() for line in r.stdout.splitlines() if line.strip()}
    return name in active_names


def get_active_wifi_connection():
    r = run(["nmcli", "-t", "-f", "DEVICE,TYPE,STATE,CONNECTION", "device", "status"])
    if r.returncode != 0:
        raise RuntimeError(f"Failed to read device status: {command_error_text(r)}")

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
    if r.returncode != 0:
        raise RuntimeError(f"Failed to read SSID for {connection_name}: {command_error_text(r)}")
    ssid = r.stdout.strip()
    if ssid:
        return ssid
    return connection_name


def get_client_target_ssid() -> str:
    if CLIENT_SSID:
        return CLIENT_SSID

    r = run(["nmcli", "-g", "802-11-wireless.ssid", "connection", "show", CLIENT_CONN])
    if r.returncode != 0:
        raise RuntimeError(f"Failed to read target SSID from {CLIENT_CONN}: {command_error_text(r)}")

    ssid = r.stdout.strip()
    if not ssid:
        raise RuntimeError(
            f"Connection {CLIENT_CONN} has no configured SSID. "
            "Set CLIENT_SSID explicitly in wifi_mode_switch.py."
        )
    return ssid


def list_visible_ssids() -> Optional[Set[str]]:
    # Use an explicit rescan so new AP availability is detected quickly.
    r = run(["nmcli", "-t", "-f", "SSID", "device", "wifi", "list", "--rescan", "yes"])
    if r.returncode != 0:
        print(f"Wi-Fi scan failed: {command_error_text(r)}")
        return None

    ssids = set()
    for line in r.stdout.splitlines():
        ssid = line.strip().replace("\\:", ":")
        if ssid:
            ssids.add(ssid)
    return ssids


def desired_mode_from_network_availability(client_ssid: str, fallback_mode: str = "hotspot") -> str:
    visible_ssids = list_visible_ssids()
    if visible_ssids is None:
        return fallback_mode
    if client_ssid in visible_ssids:
        return "client"
    return "hotspot"


def get_device_ipv4(device: str) -> str:
    if not device:
        return "N/A"

    r = run(["nmcli", "-g", "IP4.ADDRESS", "device", "show", device])
    if r.returncode != 0:
        raise RuntimeError(f"Failed to read IPv4 on {device}: {command_error_text(r)}")

    for line in r.stdout.splitlines():
        entry = line.strip()
        if entry:
            return entry.split("/", 1)[0]
    return "N/A"


def get_cpu_usage_percent() -> Optional[float]:
    global cpu_prev_total, cpu_prev_idle

    try:
        with open("/proc/stat", "r", encoding="utf-8") as f:
            first = f.readline().strip()
    except OSError:
        return None

    parts = first.split()
    if len(parts) < 5 or parts[0] != "cpu":
        return None

    try:
        values = [int(v) for v in parts[1:]]
    except ValueError:
        return None

    total = sum(values)
    idle = values[3] + (values[4] if len(values) > 4 else 0)

    if cpu_prev_total is None or cpu_prev_idle is None:
        cpu_prev_total = total
        cpu_prev_idle = idle
        return None

    total_delta = total - cpu_prev_total
    idle_delta = idle - cpu_prev_idle
    cpu_prev_total = total
    cpu_prev_idle = idle

    if total_delta <= 0:
        return None

    busy_ratio = 1.0 - (idle_delta / total_delta)
    return max(0.0, min(100.0, busy_ratio * 100.0))


def get_mem_usage_percent() -> Optional[float]:
    stats = {}
    try:
        with open("/proc/meminfo", "r", encoding="utf-8") as f:
            for line in f:
                if ":" not in line:
                    continue
                key, value = line.split(":", 1)
                fields = value.strip().split()
                if not fields:
                    continue
                try:
                    stats[key] = int(fields[0])
                except ValueError:
                    continue
    except OSError:
        return None

    total = stats.get("MemTotal")
    if not total:
        return None

    available = stats.get("MemAvailable")
    if available is None:
        # Fallback for older kernels where MemAvailable may be absent.
        available = stats.get("MemFree", 0) + stats.get("Buffers", 0) + stats.get("Cached", 0)

    used = total - available
    if used < 0:
        used = 0
    return max(0.0, min(100.0, (used / total) * 100.0))


def format_usage_line() -> str:
    cpu_pct = get_cpu_usage_percent()
    mem_pct = get_mem_usage_percent()

    cpu_text = "N/A" if cpu_pct is None else f"{cpu_pct:.0f}%"
    mem_text = "N/A" if mem_pct is None else f"{mem_pct:.0f}%"
    return f"CPU:{cpu_text} MEM:{mem_text}"


def mode_from_connection(connection_name: str) -> str:
    if connection_name == HOTSPOT_CONN:
        return "hotspot"
    if connection_name == CLIENT_CONN:
        return "client"
    return "unknown"


def update_oled_network_view(desired_mode: str, state: str = "Active"):
    if oled is None:
        return

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

    lines = [format_usage_line(), mode_header_line(displayed_mode)] + [
        f"SSID: {ssid}",
        f"IP: {ip_addr}",
    ]
    if state and state != "Active":
        lines.append(state)
    oled.show_lines(lines)


def switch_to_hotspot():
    global current_mode
    if connection_is_active(HOTSPOT_CONN):
        current_mode = "hotspot"
        update_oled_network_view("hotspot", "Active")
        return

    print("Switching to hotspot mode...")
    update_oled_network_view("hotspot", "Switching...")
    r = run(["nmcli", "connection", "up", HOTSPOT_CONN])
    if r.returncode == 0:
        current_mode = "hotspot"
        print("Hotspot mode active.")
        update_oled_network_view("hotspot", "Active")
    else:
        raise RuntimeError(f"Failed to enable hotspot: {command_error_text(r)}")


def switch_to_client():
    global current_mode
    if connection_is_active(CLIENT_CONN):
        current_mode = "client"
        update_oled_network_view("client", "Active")
        return

    print("Switching to client mode...")
    update_oled_network_view("client", "Switching...")
    r = run(["nmcli", "connection", "up", CLIENT_CONN])
    if r.returncode == 0:
        current_mode = "client"
        print("Client mode active.")
        update_oled_network_view("client", "Active")
    else:
        raise RuntimeError(f"Failed to enable client mode: {command_error_text(r)}")


def main():
    global current_mode, oled

    current_mode = None  # "hotspot" or "client"
    oled = OledStatusDisplay()
    oled.show_lines([format_usage_line(), mode_header_line("auto"), "Starting..."])

    if oled is not None:
        oled.show_lines([format_usage_line(), mode_header_line("auto"), "Waiting for NM..."])
    if not wait_for_nmcli_ready(NMCLI_READY_TIMEOUT_SEC):
        raise RuntimeError("NetworkManager not ready after timeout.")

    client_ssid = get_client_target_ssid()
    print(f"Target client SSID: {client_ssid}")

    # Set initial state on startup.
    if oled is not None:
        oled.show_lines([format_usage_line(), mode_header_line("auto"), "Scanning..."])
    desired_mode = desired_mode_from_network_availability(client_ssid, fallback_mode="hotspot")
    apply_desired_mode(desired_mode)
    last_reconcile = time.monotonic()
    last_display_refresh = last_reconcile

    while True:
        now = time.monotonic()
        if now - last_reconcile >= MODE_RECONCILE_SEC:
            desired_mode = desired_mode_from_network_availability(
                client_ssid,
                fallback_mode=current_mode or "hotspot",
            )
            apply_desired_mode(desired_mode)
            last_reconcile = now
        if now - last_display_refresh >= DISPLAY_REFRESH_SEC:
            update_oled_network_view(desired_mode, "Active")
            last_display_refresh = now
        time.sleep(POLL_SEC)

if __name__ == "__main__":
    exit_code = 0
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        exit_code = 1
        print(f"Fatal error: {exc}")
        bail_out_to_client(str(exc))
    finally:
        if oled is not None:
            oled.close()
    raise SystemExit(exit_code)
