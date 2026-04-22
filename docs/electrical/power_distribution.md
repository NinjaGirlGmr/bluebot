# Bluebot Power Distribution

## Overview

The robot is powered by a single **24V / 6A battery**. Power is distributed through three independent buck converter rails, each with a smoothing capacitor to mitigate voltage drop under load. All component power requirements are isolated per rail — motor current draw does not affect logic or compute power.

## Power Distribution Diagram

```mermaid
flowchart TD
classDef battery fill:#cc4400,stroke:#992200,stroke-width:2px,color:#fff;
classDef buck fill:#555,stroke:#333,stroke-width:1px,color:#fff;
classDef cap fill:#f8f9fa,stroke:#6c757d,stroke-width:1px,color:#333;
classDef v16 fill:#6f42c1,stroke:#4a2d8a,stroke-width:1px,color:#fff;
classDef v12 fill:#c07000,stroke:#8a5000,stroke-width:1px,color:#fff;
classDef v5 fill:#28a745,stroke:#1a6b2e,stroke-width:1px,color:#fff;
classDef periph fill:#d1ecf1,stroke:#17a2b8,stroke-width:1px,color:#000;
classDef motor fill:#fff3cd,stroke:#ffc107,stroke-width:1px,color:#000;
classDef data fill:#f8d7da,stroke:#dc3545,stroke-width:1px,color:#000;

BAT["BATTERY\n24V / 6A"]:::battery

%% ── 16V RAIL ──────────────────────────────
BUCK16["Buck Converter\n24V → 16V"]:::buck
CAP16["Smoothing Capacitor"]:::cap
JETSON["Jetson Orin Nano\n16V"]:::v16

LIDAR["Slamtec A2R8 LiDAR"]:::periph
CAM["Intel RealSense D435"]:::periph
IMU["Yahboom A471 IMU"]:::periph
OLED["OLED Display"]:::periph

%% ── 12V RAIL ──────────────────────────────
BUCK12["Buck Converter\n24V → 12V"]:::buck
CAP12["Smoothing Capacitor"]:::cap
HB1["BTS7960 H-Bridge #1\n(Motor Power 12V)"]:::v12
HB2["BTS7960 H-Bridge #2\n(Motor Power 12V)"]:::v12
MOT1["DC Encoder Motor — Left\n12V"]:::motor
MOT2["DC Encoder Motor — Right\n12V"]:::motor

%% ── 5V RAIL ───────────────────────────────
BUCK5["Buck Converter\n24V → 5V"]:::buck
CAP5["Smoothing Capacitor"]:::cap
ARD["Arduino Nano\n5V"]:::v5
HBL1["BTS7960 H-Bridge #1\nLogic 5V"]:::v5
HBL2["BTS7960 H-Bridge #2\nLogic 5V"]:::v5

%% ── POWER FLOW ────────────────────────────
BAT -->|"24V"| BUCK16
BAT -->|"24V"| BUCK12
BAT -->|"24V"| BUCK5

BUCK16 --> CAP16 -->|"16V"| JETSON
JETSON -->|"USB power + data"| LIDAR
JETSON -->|"USB power + data"| CAM
JETSON -->|"USB power + data"| IMU
JETSON -->|"power + data"| OLED

BUCK12 --> CAP12
CAP12 -->|"12V motor power"| HB1
CAP12 -->|"12V motor power"| HB2
HB1 -->|"motor drive"| MOT1
HB2 -->|"motor drive"| MOT2

BUCK5 --> CAP5
CAP5 -->|"5V"| ARD
CAP5 -->|"5V logic"| HBL1
CAP5 -->|"5V logic"| HBL2

%% ── DATA / CONTROL ────────────────────────
ARD <-->|"USB serial\n(data only, no power)"| JETSON
ARD -->|"PWM signal"| HB1
ARD -->|"PWM signal"| HB2
MOT1 -->|"encoder data"| ARD
MOT2 -->|"encoder data"| ARD
```

## Rail Summary

| Rail | Voltage | Buck Converter | Consumers |
|---|---|---|---|
| Compute | 16V | 24V → 16V | Jetson Orin Nano (+ downstream USB peripherals) |
| Drive | 12V | 24V → 12V | BTS7960 H-Bridge motor power (×2) → DC motors (×2) |
| Logic | 5V | 24V → 5V | Arduino Nano, BTS7960 H-Bridge logic power (×2) |

## Notes

- All three rails include smoothing capacitors to absorb transient current draw, particularly from motor start/stop events on the 12V rail.
- The Arduino Nano is powered exclusively from the 5V rail. Its USB connection to the Jetson carries serial data only — no bus power is drawn from the Jetson.
- The Arduino sends PWM signals to each BTS7960 H-Bridge to control motor speed and direction. Encoder feedback from both drive motors is wired directly back to the Arduino for odometry.
- Motor power (12V) and H-Bridge logic power (5V) are on separate rails so motor switching transients cannot affect Arduino or Jetson stability.
- The LiDAR, RealSense D435, and IMU are all connected to the Jetson via USB, which provides both power and data. The Jetson's onboard regulators supply USB bus power from its 16V input.
