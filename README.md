# Autonomous Maze Robot with Camera Vision

An autonomous robot that finds its way through a maze and counts coloured cubes with a camera. A Raspberry Pi running embedded Linux handles the vision; an Arduino Mega reads the wall sensors and the gyroscope and drives the motors. The Pi sends codes to the Arduino over a simple 2-bit GPIO link.

Built for the **ENET'Com Sfax robotics competition**, Autonomous category (2025).

<!-- TODO: add a photo of the robot -->
<!-- ![Robot](docs/images/robot.jpg) -->

## Videos

| Autonomous run | Cube counting (Raspberry Pi camera) |
|---|---|
| [![Autonomous run](https://img.youtube.com/vi/klpP9VOge3w/hqdefault.jpg)](https://youtu.be/klpP9VOge3w) | [![Cube counting](https://img.youtube.com/vi/V6KUgvmmu3I/hqdefault.jpg)](https://youtu.be/V6KUgvmmu3I) |
| [▶️ Watch on YouTube](https://youtu.be/klpP9VOge3w) | [▶️ Watch on YouTube](https://youtu.be/V6KUgvmmu3I) |

## Architecture

```
camera ──► Raspberry Pi (embedded Linux)
           cube detection and counting (OpenCV)
                     │
                     │  2-bit GPIO link (codes 0 to 3)
                     ▼
1 × ultrasonic ──┐
2 × infrared   ──┼──► Arduino Mega 2560 Pro ──► 2 × TB6612FNG ──► motors
MPU6050 gyro   ──┘    wall sensing, gyro PID moves and turns, OLED
```

- **Maze navigation**: one ultrasonic sensor and two infrared sensors detect the walls; a gyroscope corrects the heading so the robot drives straight and turns accurately.
- **Vision**: the Pi camera detects cubes of a chosen colour and counts them.
- **Split design**: vision on the Raspberry Pi, sensors and real-time motor control on the Arduino.

## Cube counter

[`raspberry_pi/cube_counter.py`](raspberry_pi/cube_counter.py) counts how many cubes of one colour (red, green or blue) the robot passes. Each cube is tracked from frame to frame, so a cube that stays in view for a long time is counted only once.

**Pipeline, for each camera frame:**

```
frame ──► HSV ──► colour mask ──► noise removal ──► contours ──► merged boxes
                                                                      │
             count += 1  ◄──  seen 5 frames in a row  ◄──  matched to the previous frame
```

1. The frame is converted to HSV and the pixels of the chosen colour are kept. Red wraps around hue 0, so it uses two ranges.
2. Morphological opening and closing remove noise and fill small holes.
3. Blobs smaller than 1% of the image are ignored, and pieces of the same cube are merged into one box.
4. Each cube is matched to the box it overlaps most in the previous frame (simple tracking).
5. A cube is counted once it has been seen for 5 frames in a row, and forgotten after 10 frames out of view.

### Install (Raspberry Pi OS Bookworm)

```bash
sudo apt update
sudo apt install -y python3-picamera2 python3-opencv
```

On a PC (for testing on recorded videos): `pip install -r raspberry_pi/requirements.txt`

### Usage

```bash
python3 raspberry_pi/cube_counter.py                  # asks: Enter the color (R, G, or B)
python3 raspberry_pi/cube_counter.py --color G        # count green cubes
python3 raspberry_pi/cube_counter.py --color B --show # show the camera with outlines (needs a screen)
python3 raspberry_pi/cube_counter.py --color R --video run.mkv   # test on a recorded video
```

Stop with **Ctrl+C** (or **q** in the video window). At the end it prints, for example: `Total Green cubes detected: 3`

### Tuning

All settings are at the top of `cube_counter.py`:

| Setting | Meaning |
|---|---|
| `COLOR_RANGES` | HSV range for each colour; adjust if the lighting changes the colours |
| `MIN_AREA_RATIO` | Smallest blob that counts as a cube (fraction of the image) |
| `CONFIRM_FRAMES` | Frames a cube must be visible before it is counted |
| `MAX_MISSED_FRAMES` | Frames a cube can disappear before it is forgotten |
| `MATCH_MARGIN` | How far a cube can move between frames and still be the same cube |

## Pi to Arduino link (2-bit GPIO)

The Raspberry Pi sends information to the Arduino on **two wires, one bit each**, so it can send a code from 0 to 3. What each code means depends on the mission (for example a colour, a number of cubes or a direction).

| Code | BIT1: Pi GPIO 27 → Mega pin 25 | BIT0: Pi GPIO 17 → Mega pin 24 |
|---|---|---|
| 0 | LOW | LOW |
| 1 | LOW | HIGH |
| 2 | HIGH | LOW |
| 3 | HIGH | HIGH |

**How it works:**

- The code is a **level**, not a message: the Pi keeps it on the wires until it sends a new one, and the Arduino reads it whenever its program needs it. There is no protocol to parse and no timing to respect on the Pi side.
- The Pi changes the two bits one after the other, so for a few microseconds the wires can show a wrong code. The Arduino's `readPiCode()` reads the lines twice, 2 ms apart, and only returns a value once both reads agree.
- Wiring: the two signal wires plus a **common ground**. The Pi's 3.3 V HIGH is read as HIGH by the Mega's 5 V inputs, so no level shifter is needed in this direction. The link only goes from the Pi to the Arduino: never connect a Mega output to a Pi GPIO without a level shifter.
- Optional: two 10 kΩ pull-down resistors on the Mega side keep the code at 0 while the Pi is booting.

**Code:**

| Side | File | Functions |
|---|---|---|
| Raspberry Pi | [`raspberry_pi/pi_link.py`](raspberry_pi/pi_link.py) | `PiLink().send(code)` |
| Arduino | [`arduino/maze_robot/Link.ino`](arduino/maze_robot/Link.ino) | `readPiCode()`, `waitForPiCode(code, timeoutMs)` |

**Check the wiring:** run `python3 raspberry_pi/pi_link.py --cycle` on the Pi. The Arduino's sensor screen shows `Pi code: 0, 1, 2, 3` changing every second.

Example on the Pi:

```python
from pi_link import PiLink

link = PiLink()
link.send(2)   # the Arduino now reads 2
```

Example on the Arduino:

```cpp
if (waitForPiCode(3, 5000)) {   // wait up to 5 s for code 3
  // the mission decides what code 3 means
}
```

## Arduino firmware

[`arduino/maze_robot/`](arduino/maze_robot/) runs on an **Arduino Mega 2560 Pro** and contains the robot's low-level building blocks:

| File | Contents |
|---|---|
| `maze_robot.ino` | Pins, globals, `setup()`, `loop()`, startup sequence (gyro calibration) |
| `Gyro.ino` | MPU6050 with its DMP: heading reading, PID straight moves (`gyroDriveStraightPID`) and PID turns to an angle (`gyroTurnPID`) |
| `Link.ino` | 2-bit GPIO link from the Raspberry Pi: `readPiCode()`, `waitForPiCode()` |
| `Utils.ino` | Motors (TB6612FNG), ultrasonic distance in cm (`readDistanceCm`), left / right infrared proximity sensors (`readProximityLeft/Right`), RGB LED, buzzer, OLED |

After the gyro calibration, `loop()` shows the heading, the front distance and both side walls on the OLED, which is used to check the sensors before a run. The competition's maze sequence is not included in this version.

**Gyro turns, how they work:** the MPU6050's on-chip motion processor (DMP) gives the yaw angle; `gyroTurnPID` spins the robot until the heading error to the target angle is close to zero, with a PD correction on the motor speeds. A second version (`gyroTurnPID_v2`) takes the shortest way around when the angle wraps past ±180°.

### Build and flash

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) and these libraries: Adafruit SSD1306, Adafruit GFX, SparkFun TB6612FNG, I2Cdevlib MPU6050.
2. Open `arduino/maze_robot/maze_robot.ino`, select **Arduino Mega or Mega 2560**, and upload.

## Repository layout

```
raspberry_pi/          Raspberry Pi: cube counter (OpenCV + Picamera2), 2-bit link to the Arduino
arduino/maze_robot/    Arduino Mega: motors, gyro moves and turns, wall sensors
```

**Not included in this version:** the competition's mission program (the maze sequence and what each link code means during the run). This repository contains the vision pipeline, the Pi to Arduino link and the Arduino building blocks (sensors, gyro moves and turns).

## Tech

Raspberry Pi, embedded Linux, Python, OpenCV, Picamera2, gpiozero, HSV colour segmentation, object tracking, Arduino Mega 2560 Pro, C++, MPU6050 (DMP), PID control, GPIO link, ultrasonic and infrared sensors, gyroscope
