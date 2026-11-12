# ESP32 RMT Stepper & Motion Control Firmware — Eurobot PAMI (WIP)

**Author:** Martin Dutang — *CentraleSupélec & McGill University (Bachelor of Global Engineering)*  
**Context:** *Club Tech CentraleSupélec — Coupe de France de Robotique (Eurobot)*  
**Status:** `Work in Progress` *(Open-loop hardware RMT stepper layer complete; closed-loop odometry & polar `GoTo` in progress)*  
**Stack:** `Embedded C` · `ESP-IDF` · `FreeRTOS` · `ESP32 RMT Peripheral`

---

## Overview

In the **Coupe de France de Robotique (Eurobot)**, **PAMIs** (*Petits Actionneurs Mobiles Indépendants*) are compact autonomous secondary robots deployed during the final seconds of a match.

This repository contains my low-level **ESP32 motion-control firmware** written in C on top of **ESP-IDF** and **FreeRTOS**, using the ESP32's dedicated **RMT (Remote Control Transceiver) TX hardware peripheral** to drive dual microstepping stepper motors smoothly without CPU bit-banging jitter.

---

## Hardware & Kinematic Configuration (`robot_types.h`, `motor.h`)

| Parameter | Value | Description |
| :--- | :--- | :--- |
| **MCU / Framework** | `ESP32` (`ESP-IDF` + `FreeRTOS`) | Hardware pulse generation via `driver/rmt_tx.h` |
| **Pinout (`STEP` / `DIR`)** | Left: `GPIO 11 / 10` · Right: `GPIO 7 / 8` | Dual stepper driver step/direction interface |
| **RMT Clock Resolution** | $1\text{ MHz}$ ($1\text{ }\mu\text{s}$ tick, $50\text{ }\mu\text{s}$ high pulse) | Infinite hardware loop mode (`.loop_count = -1`) |
| **Stepper Resolution** | $1600\text{ microsteps/rev}$ | $\Delta s = \frac{2\pi R}{1600} \approx 98.17\text{ }\mu\text{m/step}$ |
| **Chassis Geometry** | $R = 25\text{ mm}$, $b = 60\text{ mm}$ ($120\text{ mm}$ track) | Differential-drive PAMI footprint |

### Velocity-to-Pulse Period Mapping (`motor.c`)

Linear wheel speed $v$ ($\text{m/s}$) and chassis angular velocity $\omega$ ($\text{rad/s}$) map to RMT symbol periods via:

$$
T_{\text{ticks}}(v) = \frac{f_{\text{RMT}} \cdot 2\pi R}{N_{\text{steps}} \, v}, \qquad v_{R,L} = \pm \, \omega \, b \, k_{\text{rot}}
$$

---

## Current File Breakdown

- **`robot_types.h`** — Core data structures for chassis geometry, wheel ticks/velocities (`Wheel_ticks`, `Wheel_speed`), planar robot pose (`Pose`: $x, y, \theta$), and closed-loop controller state (`Asserv_t`, `Asserv_status`).
- **`motor.h` / `motor.c`** — ESP32 RMT TX channel initialization, independent left/right step-frequency control (`ms_to_pd`, `pd_to_ms`), direction management, in-place rotation (`motor_turn`), and linear displacement (`traveling`).
- **`main.c`** — ESP-IDF `app_main()` GPIO setup and open-loop hardware bring-up test sequence.

---

## Development Roadmap

- [x] Core data structures (`Pose`, `Wheel_ticks`, `Wheel_speed`, `Asserv_t`)
- [x] Dual-channel ESP32 RMT TX infinite-loop pulse generator (`1 MHz`)
- [x] Open-loop differential primitives (`traveling(distance)`, `motor_turn(theta)`)
- [ ] Pulse-width (`UP_DURATION`) period offset & 15-bit RMT low-speed clamping refinement
- [ ] Differential-drive pose integration from step/encoder counts (`odometry.c`)
- [ ] Closed-loop polar `GoTo(x, y, theta)` controller (`asserv.c`) & match `tirette` sequencing
