# M5Core2 Mecanum Trash Can - Complete Setup & Calibration Guide

[![Powered by cl0ud](https://img.shields.io/badge/Powered%20by-cl0ud-00E5FF?style=for-the-badge&logoColor=0B0F19&labelColor=0B0F19)](https://github.com/Ojas-sta/cl0ud-brand-engineering)
[![Co-engineered by Ojas Singh](https://img.shields.io/badge/Co--engineered%20by-Ojas%20Singh-1E3A8A?style=for-the-badge&logoColor=white&labelColor=0B0F19)](https://github.com/Ojas-sta)

## 1. Drivetrain Architecture & Mecanum Wheel Orientation

A 4x4 Mecanum drive operates by Vector Force Addition. For the omnidirectional "swerve-style" movement to function properly, the 4 Mecanum wheels **must be mounted in the correct physical orientation**.

### Crucial Roller Check:
Look down at your trash can robot from the top:
- The rollers touching the floor must form an **"X"** shape.
- When viewing the rollers from the top of the robot, the rollers point toward the center in a **diamond / "O"** shape:

```
                  FRONT (Trash Can Forward)
         [M0: Front-Left]       [M1: Front-Right]
           Roller: \                 Roller: /
                    \               /
                     \             /
                      \           /
                       \         /
                        \       /
           Roller: /                 Roller: \
         [M2: Rear-Left ]       [M3: Rear-Right ]
                   REAR (Trash Can Back)
```

- **Front-Left (CH 0)**: Rollers angled top-left to bottom-right (`\`)
- **Front-Right (CH 1)**: Rollers angled top-right to bottom-left (`/`)
- **Rear-Left (CH 2)**: Rollers angled bottom-left to top-right (`/`)
- **Rear-Right (CH 3)**: Rollers angled bottom-right to top-left (`\`)

*(If the robot spins when you try to strafe sideways, two of the wheels are swapped on the wrong sides).*

---

## 2. RadioMaster Pocket Configuration (EdgeTX)

1. Turn ON the **RadioMaster Pocket**.
2. Create or select a clean RC model profile (e.g. named `TRASHCAN` or `MECANUM`).
3. Standard Channel Layout (Mode 2):
   - **CH1**: Roll / Aileron (Right Stick Horizontal) -> **Strafe Left / Right**
   - **CH2**: Pitch / Elevator (Right Stick Vertical) -> **Forward / Reverse**
   - **CH3**: Throttle (Left Stick Vertical) -> **Master Speed / Power Throttle**
   - **CH4**: Rudder / Yaw (Left Stick Horizontal) -> **Yaw / Rotate Left / Right**
4. Plug the **top USB-C port** of the RadioMaster Pocket into your laptop.
5. On the RadioMaster LCD, select:
   **`USB Joystick (HID)`**
6. Windows will recognize the radio as a DirectInput USB Game Controller.

---

## 3. Laptop Controller Setup & Verification

Open a terminal or command prompt in `c:\Users\shashankj\Documents\m5RASHCAN`:

### Step 3.1: Test & Calibrate Radio Sticks
Double-click `run_tester.bat` or run:
```powershell
.venv\Scripts\python.exe laptop_controller\joystick_tester.py
```
Move your sticks. You will see real-time graphical bars for each axis.

To run the guided auto-calibration wizard:
```powershell
.venv\Scripts\python.exe laptop_controller\joystick_tester.py --calibrate
```
Follow the on-screen prompts:
1. Push Right Stick Right (Strafe X)
2. Push Right Stick Forward (Forward Y)
3. Push Left Stick Up (Throttle Y)
4. Push Left Stick Right (Yaw X)
5. Assign optional ARM switch.
This automatically writes your exact axis IDs to `laptop_controller/config.json`.

---

## 4. M5Core2 Firmware Flashing

### Option A: Using Arduino IDE
1. Open Arduino IDE.
2. Install **ESP32 Board Package** (`esp32` by Espressif Systems).
3. In **Library Manager**, install:
   - `M5Unified` (by M5Stack)
   - `M5GFX` (by M5Stack)
   *(Note: The `M5Module4EncoderMotor` driver files are already bundled right in the `m5core2_firmware/` folder!)*
4. Open `m5core2_firmware/m5core2_firmware.ino`.
5. Select Board: **M5Stack Core2** (or `M5Core2`).
6. Port: Select the COM port of your connected M5Core2.
7. Click **Upload**.

### Option B: Using PlatformIO (VS Code)
1. Open the folder `m5core2_firmware` in VS Code with PlatformIO extension installed.
2. Click **Build** and **Upload**.

---

## 5. Driving the Trash Can!

1. Power on the M5Core2 and the 4-channel motor stack (external battery connected to motor screw terminals).
2. The M5Core2 screen will display:
   - Connecting to `0xMakers_Dev`...
   - WiFi Connected! `IP: 192.168.x.x`
3. Make sure your Laptop is connected to the same WiFi: `0xMakers_Dev`.
4. Launch the UDP bridge on your laptop:
   Double-click `run_bridge.bat` or run:
   ```powershell
   .venv\Scripts\python.exe laptop_controller\radio_bridge.py
   ```
5. The bridge will automatically discover the M5Core2 over the network and begin streaming control packets at 40 Hz!

### Control Scheme in Action:
- **Right Stick (2D Planar Direction)**:
  - Push Forward: Drives straight forward.
  - Push Left/Right: Strafes purely sideways without rotating!
  - Push Diagonally: Robot glides diagonally in any 360° heading.
- **Left Stick (Speed & Rotation)**:
  - **Left-Y (Throttle)**: Scales your master driving speed smoothly from 0% (stopped) to 100% (full speed).
  - **Left-X (Yaw)**: Rotates the trash can clockwise or counter-clockwise.
  - You can translate with the right stick while rotating with the left stick simultaneously!
- **Emergency Stop**:
  - Tap anywhere on the M5Core2 touchscreen to immediately freeze all motors. Tap again to resume.

---

## 6. Autonomous Person-Tracking & Trash Collection Workflow

Binzo can operate fully autonomously to patrol a room along a taped line track, detect nearby humans, offer its bin, and return to the line loop:

```
+-------------------------------------------------------------------+
|                        [AUTO_LINE_PATROL]                         |
|           Follows black/white tape line loop via TCRT5000         |
+---------------------------------+---------------------------------+
                                  |
                   Person Detected by M5StickV Camera
                  (cx centered, confidence > 65%, bbox > 30px)
                                  v
+-------------------------------------------------------------------+
|                       [AUTO_APPROACH_PERSON]                      |
|      Enters encoder odometry tracking; steers towards person      |
|      using proportional visual servoing until target bbox reached |
+---------------------------------+---------------------------------+
                                  |
                       Target Distance Reached
                                  v
+-------------------------------------------------------------------+
|                        [AUTO_OPEN_MOUTH]                          |
|         Commands Servo Ch 0 on 4EncoderMotor Module to 95°        |
|             Screen displays open-mouthed FEED ME face             |
+---------------------------------+---------------------------------+
                                  |
                                  v
+-------------------------------------------------------------------+
|                        [AUTO_AWAIT_TRASH]                         |
|           Waits 5.0 seconds for user to deposit trash             |
+---------------------------------+---------------------------------+
                                  |
                                  v
+-------------------------------------------------------------------+
|                        [AUTO_CLOSE_MOUTH]                         |
|         Commands Servo Ch 0 back to 0° (lid closed)               |
|            Screen displays smiling THANK YOU face                 |
+---------------------------------+---------------------------------+
                                  |
                                  v
+-------------------------------------------------------------------+
|                        [AUTO_RETURN_LINE]                         |
|    Drives reverse/return path using encoder tick accumulation     |
+---------------------------------+---------------------------------+
                                  |
                       Encoder Target Reached
                                  v
+-------------------------------------------------------------------+
|                      [AUTO_REACQUIRE_LINE]                        |
|       Rotates / creeps until TCRT5000 re-locks onto tape line     |
+---------------------------------+---------------------------------+
                                  |
                         Line Detected (TCRT5000)
                                  v
                             (Back to Loop)

```

### Hardware Connections:
| Subsystem | Port / Pins | Function |
|---|---|---|
| **M5StickV Camera** | Port C (Grove Blue: GPIO 13 RX, 14 TX) | 16.5 FPS Person Detector via YOLOv2 UART stream |
| **Line Follower** | Port B (Grove Black: GPIO 26 OneWire)| 5-Channel Multiple Line Follower / TCRT5000 |
| **Mouth Servo** | 4EncoderMotor Servo Ch 0 (I2C addr 0x24) | MG90S servo actuates bin lid open/close |
| **Mecanum Drivetrain** | 4EncoderMotor Motor Ch 0-3 (I2C addr 0x24) | 4-channel closed-loop encoder DC motors |

---

## 🤝 Engineering Attribution & Credits

- **Original Project & Concept:** Binzo Team (Shashank Jangid & contributors)
- **M5Core2 Mecanum Architecture & FSM:** Co-engineered by **Ojas Singh** | Powered by **cl0ud** ([cl0ud Brand Engineering](https://github.com/Ojas-sta/cl0ud-brand-engineering))
- **Vision Subsystem (K210 YOLOv2 Tracking):** Co-engineered by **Ojas Singh** | Powered by **cl0ud**
- **5-Channel Line Follower & Odometry:** Co-engineered by **Ojas Singh** | Powered by **cl0ud**

<div align="center">
  <a href="https://github.com/Ojas-sta/cl0ud-brand-engineering">
    <img src="https://raw.githubusercontent.com/Ojas-sta/cl0ud-brand-engineering/main/assets/badges/cl0ud-credit-card.svg" alt="Powered by cl0ud | Co-engineered by Ojas Singh" width="100%" style="max-width: 820px;" />
  </a>
</div>

