# `wifi_mode_switch.py`

Automatic Wi-Fi mode manager for Jetson, with optional 0.96" I2C OLED status display.

## What it does

- Uses `nmcli` to scan for a known client SSID.
- If the client SSID is visible, brings up client mode (`CLIENT_CONN`).
- If the client SSID is not visible, brings up hotspot mode (`HOTSPOT_CONN`).
- Re-checks periodically so it can switch back to client when the network returns.
- Shows status on SSD1306 OLED over I2C (startup, switching, active, failure).
- Designed to run continuously as a boot service.

## Files

- Script: `/ssd/ros2_ws/scripts/wifi_mode_switch.py`
- Service unit: `/ssd/ros2_ws/scripts/wifi_mode_switch.service`

## Hardware wiring

### Optional: 0.96" OLED (SSD1306 I2C, 128x64)

- `VCC` -> Jetson pin `1` (`3.3V`)
- `GND` -> Jetson pin `6` (`GND`)
- `SDA` -> Jetson pin `3` (`I2C SDA`)
- `SCL` -> Jetson pin `5` (`I2C SCL`)

Notes:
- Most 0.96" SSD1306 modules use I2C address `0x3C` (default in script).
- Do not power a 3.3V-only OLED from 5V.
- The top yellow band is used for one large `CPU/MEM` utilization line.
- Network fields are rendered in the blue section.

## Software prerequisites

- `python3`
- `python3-pil` (Pillow)
- `network-manager` (`nmcli` command available)
- I2C enabled on Jetson (`/dev/i2c-*` present)

Quick checks:

```bash
python3 -c "import PIL; print('ok')"
nmcli --version
ls /dev/i2c-*
```

## Configuration

Edit constants at the top of the script if needed:

- `HOTSPOT_CONN`
- `CLIENT_CONN`
- `CLIENT_SSID` (`None` = auto-detect from `CLIENT_CONN`)
- `POLL_SEC`
- `NMCLI_READY_TIMEOUT_SEC`, `MODE_RECONCILE_SEC`
- `OLED_ENABLED`
- `OLED_I2C_BUS` (`None` = auto-detect from candidates)
- `OLED_I2C_ADDR` (default `0x3C`)
- `OLED_TOP_FONT_SIZE`, `OLED_BODY_FONT_SIZE`

SSID selection details:
- If `CLIENT_SSID` is set, that exact SSID is used for availability checks.
- If `CLIENT_SSID` is `None`, the script reads the SSID from the `CLIENT_CONN` profile (`802-11-wireless.ssid`).

## Run manually

```bash
python3 /ssd/ros2_ws/scripts/wifi_mode_switch.py
```

Stop with `Ctrl+C`.

## Run at boot (systemd)

Install and enable:

```bash
sudo cp /ssd/ros2_ws/scripts/wifi_mode_switch.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now wifi_mode_switch.service
```

Check status and logs:

```bash
systemctl status wifi_mode_switch.service
journalctl -u wifi_mode_switch.service -f
```

## OLED status screens

- Standard view (refreshed every `DISPLAY_REFRESH_SEC`):
- `CPU:<util> MEM:<util>` (yellow band)
- `Mode: <hotspot|client|auto>`
- `SSID: <active-ssid>`
- `IP: <jetson-ipv4>`
- Extra state line appears during transitions:
- `Starting...`
- `Waiting for NM...`
- `Scanning...`
- `Switching...`
- `FAILED`

## Troubleshooting

1. No OLED output
Verify wiring and power.
Verify I2C device exists: `ls /dev/i2c-*`.
Scan bus: `sudo i2cdetect -y 1` and confirm `0x3C` (or update `OLED_I2C_ADDR`).

2. Service starts but always stays in hotspot mode
Verify `CLIENT_CONN` exists: `nmcli connection show`.
Verify target SSID matches exactly.
Check scan output manually: `nmcli -t -f SSID device wifi list --rescan yes`.

3. Service starts but WiFi does not switch as expected
Verify connection names exactly match `nmcli connection show`.
Check logs: `journalctl -u wifi_mode_switch.service -n 200`.

4. Script exits immediately
Check Python dependencies are installed.
Ensure `nmcli` is available and NetworkManager is running.

## Behavior notes

- On startup, the script scans for the client SSID and applies client or hotspot mode.
- The script waits for NetworkManager readiness, then retries periodically (`MODE_RECONCILE_SEC`) to recover from transient boot-time failures.
- If an SSID scan fails temporarily, it keeps the current mode and retries on the next reconcile cycle.
