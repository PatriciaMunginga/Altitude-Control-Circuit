# mini-drone-altitude-control

Arduino-based altitude control circuit for a mini-drone using an ultrasonic sensor and PID controller.

---

## Table of Contents

- [Overview](#overview)
- [Hardware Components](#hardware-components)
- [Circuit](#circuit)
- [How It Works](#how-it-works)
- [PID Controller](#pid-controller)
- [Configuration](#configuration)
- [Setup & Usage](#setup--usage)
- [Serial Monitor Output](#serial-monitor-output)

---

## Overview

This project implements a closed-loop altitude control system for a mini-drone. An ultrasonic sensor measures the drone's current height, a potentiometer sets the target height, and a PID controller adjusts motor speed via two ESCs to maintain the desired altitude.

---

## Hardware Components

| Component | Description |
|---|---|
| Arduino Uno | Microcontroller |
| Ultrasonic Sensor (HC-SR04) | Measures altitude via echo signal |
| ESC x2 | Electronic Speed Controllers for motor control |
| Brushless Motors x2 | Drone propulsion |
| Potentiometer | Sets target altitude |
| Push Button | Start/stop control |

---

## Circuit

| Component | Arduino Pin |
|---|---|
| Ultrasonic Trigger | Pin 7 |
| Ultrasonic Echo | Pin 8 |
| ESC 1 | Pin 9 |
| ESC 2 | Pin 10 |
| Push Button | Pin 3 |
| Potentiometer | A0 |

---

## How It Works

1. **Start** — Press the button to arm the drone. The ESCs go through a calibration sequence on startup.
2. **Liftoff** — Motor speed ramps up gradually for a smooth takeoff.
3. **Altitude sensing** — The ultrasonic sensor fires a 10µs pulse and measures the echo return time, which is converted to a distance in centimetres.
4. **Target setting** — The potentiometer maps to a target height between `min_height` (10cm) and `max_height` (50cm).
5. **PID control** — The controller computes the error between target and measured height and adjusts the PWM signal sent to both ESCs every 100ms.
6. **Stop** — Press the button again to trigger a controlled landing.

---

## PID Controller

The control output is calculated as:

```
output = (Kp × error) + (Ki × errSum × SampleTime) + (Kd × dErr / SampleTime) + PWM_offset
```

| Parameter | Value | Description |
|---|---|---|
| `Kp` | 3 | Proportional gain |
| `Ki` | 0 | Integral gain |
| `Kd` | 0 | Derivative gain |
| `SampleTime` | 100ms | Control loop interval |
| `PWM_offset` | 50 | Hover offset to keep drone airborne |

> The PWM signal is clamped between `0` and `180` to stay within safe ESC range.

---

## Configuration

The following constants can be tuned at the top of the sketch:

```cpp
const int min_height = 10;  // minimum target height in cm
const int max_height = 50;  // maximum target height in cm
const int ramp_time  = 40;  // motor ramp up/down speed in ms
int SampleTime       = 100; // PID update interval in ms
double Kp = 3;              // proportional gain
double Ki = 0;              // integral gain
double Kd = 0;              // derivative gain
int PWM_offset       = 50;  // hover PWM value
```

---

## Setup & Usage

1. Wire the components according to the [circuit table](#circuit) above.
2. Open `mini_drone_altitude_control.ino` in the Arduino IDE.
3. Select **Arduino Uno** as the board and the correct COM port.
4. Upload the sketch.
5. Open the Serial Monitor at **9600 baud** to observe the ESC calibration sequence.
6. Press the button to start the drone.
7. Adjust the potentiometer to change the target altitude.
8. Press the button again to land.

> **Warning:** Test in a safe, open area. Keep hands clear of propellers during operation.

---

## Serial Monitor Output

```
ESC calibration process
Starting...
Now writing maximum output
Now writing minimum output
Press button to start minidrone

Target distance: 30
Measured distance: 28
Error: 2
PWM: 56
Button Status: 1
```

---

## Reference

Ben Finio, Science Buddies, 2021.

## Author

Patricia Munginga, 2023
