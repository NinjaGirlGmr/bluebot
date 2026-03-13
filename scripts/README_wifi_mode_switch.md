# `wifi_mode_switch.py`

GPIO-driven WiFi mode switch for Jetson, with optional 0.96" I2C OLED status display.

## What it does

- Reads a hardware switch on Jetson BOARD pin `7` (`GPIO4`, physical pin 7).
- Interprets `GPIO LOW` as hotspot mode (`BluebotHotspot`).
- Interprets `GPIO HIGH` as client mode (`Bazinga5`).
- Uses `nmcli` to bring the selected NetworkManager connection up.
- Shows status on SSD1306 OLED over I2C (startup, switching, active, failure).
- Designed to run continuously as a boot service.

## Files

- Script: `/ssd/ros2_ws/scripts/wifi_mode_switch.py`
- Service unit: `/ssd/ros2_ws/scripts/wifi_mode_switch.service`

## Hardware wiring

### 1) Mode switch input (already in script)

- Signal -> Jetson physical pin `7` (`GPIO_PIN = 7`)
- GND -> any ground pin
- Internal pull-up is enabled in software, so grounding the signal reads `LOW`.

### 2) 0.96" OLED (SSD1306 I2C, 128x64)

- `VCC` -> Jetson pin `1` (`3.3V`)
- `GND` -> Jetson pin `6` (`GND`)
- `SDA` -> Jetson pin `3` (`I2C SDA`)
- `SCL` -> Jetson pin `5` (`I2C SCL`)

Notes:
- Most 0.96" SSD1306 modules use I2C address `0x3C` (default in script).
- Do not power a 3.3V-only OLED from 5V.

## Software prerequisites

- `python3`
- `python3-pil` (Pillow)
- `python3-jetson-gpio`
- `network-manager` (`nmcli` command available)
- I2C enabled on Jetson (`/dev/i2c-*` present)

Quick checks:

```bash
python3 -c "import Jetson.GPIO, PIL; print('ok')"
nmcli --version
ls /dev/i2c-*
```

## Configuration

Edit constants at the top of the script if needed:

- `GPIO_PIN`
- `HOTSPOT_CONN`
- `CLIENT_CONN`
- `POLL_SEC`, `DEBOUNCE_SEC`
- `NMCLI_READY_TIMEOUT_SEC`, `MODE_RECONCILE_SEC`
- `OLED_ENABLED`
- `OLED_I2C_BUS` (`None` = auto-detect from candidates)
- `OLED_I2C_ADDR` (default `0x3C`)

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
- `Mode: hotspot|client`
- `SSID: <active-ssid>`
- `IP: <jetson-ipv4>`
- Extra state line appears during transitions:
- `Starting...`
- `Waiting for NM...`
- `Switching...`
- `FAILED`
- `NM not ready`

## Troubleshooting

1. No OLED output
Verify wiring and power.
Verify I2C device exists: `ls /dev/i2c-*`.
Scan bus: `sudo i2cdetect -y 1` and confirm `0x3C` (or update `OLED_I2C_ADDR`).

2. Service starts but WiFi does not switch
Verify connection names exactly match `nmcli connection show`.
Check logs: `journalctl -u wifi_mode_switch.service -n 200`.

3. Script exits immediately
Check Python dependencies are installed.
Ensure `nmcli` is available and NetworkManager is running.

## Behavior notes

- On startup, desired mode is read from the switch and applied.
- The script waits for NetworkManager readiness, then retries periodically (`MODE_RECONCILE_SEC`) to recover from transient boot-time failures.
