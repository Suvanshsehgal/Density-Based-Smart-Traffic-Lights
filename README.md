#  Density-Based Smart Traffic Light System

A smart traffic management solution that dynamically adjusts signal timings based on real-time vehicle density using **YOLO** (You Only Look Once) object detection and a threshold-based logic model. Designed to reduce congestion and enhance urban mobility.

---

##  Project Preview



![WhatsApp Image 2025-05-09 at 10 44 31_9cc15fc3](https://github.com/user-attachments/assets/27cbe2d2-4ed6-4e0e-9e85-5d46b010a314)
![Screenshot_2025-05-09_104301 1](https://github.com/user-attachments/assets/30f64f51-809f-4df2-be54-ff406c5d5dd4)
---

## Overview

This system detects the number of vehicles on the road using **YOLO-based object detection** and accordingly determines how long the green light should remain active for a lane.

- **Real-time vehicle detection**
- **1 second per vehicle logic**
- **Machine learning integration**
- **Camera/ESP32-CAM input**
- **Hardware interaction with traffic lights**

---

## How It Works

1. **Video Feed Input**: Uses a webcam or ESP32-CAM.
2. **Object Detection**: YOLO detects and counts the number of vehicles.
3. **Timer Logic**: Each vehicle gets 1 second of green light time.
4. **Traffic Signal Control**: Green light duration is set dynamically.
5. **Hardware**: Sends control signals to Arduino or microcontroller to switch LEDs/relays.

---

## Tech Stack

| Category       | Tools Used              |
|----------------|--------------------------|
| Language       | Python                   |
| AI Model       | YOLOv5 (trained or pretrained) |
| Libraries      | OpenCV, PyTorch, NumPy   |
| Hardware       | ESP32-CAM, Arduino       |
| Communication  | pySerial (Python-Arduino) |
| IDEs           | VS Code, Arduino IDE     |

---

