# Autonomous Maze Robot with Camera Vision

An autonomous robot that finds its way through a maze and counts coloured cubes with a camera. A Raspberry Pi running embedded Linux handles the sensors and the vision; an Arduino drives the motors through a custom UART protocol.

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
camera            ──┐
1 × ultrasonic    ──┤     Raspberry Pi (embedded Linux)
2 × infrared      ──┼──►  cube detection and counting (OpenCV)
gyroscope         ──┘     wall detection, navigation, heading correction
                                     │
                                     │  custom UART protocol
                                     ▼
                          Arduino ──► motor drivers ──► motors
```

- **Maze navigation**: one ultrasonic sensor and two infrared sensors detect the walls; a gyroscope corrects the heading so the robot drives straight and turns accurately.
- **Vision**: the Pi camera detects cubes of a chosen colour and counts them.
- **Split design**: high-level logic on the Raspberry Pi, real-time motor control on the Arduino.

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

## Repository layout

```
raspberry_pi/   Raspberry Pi code: cube counter (OpenCV + Picamera2)
```

<!-- TODO: add the navigation code (raspberry_pi/) and the Arduino motor sketch (arduino/) with the UART protocol -->

## Tech

Raspberry Pi, embedded Linux, Python, OpenCV, Picamera2, HSV colour segmentation, object tracking, Arduino, C++, UART, ultrasonic and infrared sensors, gyroscope
