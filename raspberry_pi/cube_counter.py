#!/usr/bin/env python3
"""
Cube counter for Raspberry Pi.

Counts how many cubes of one color (R, G or B) the camera passes.
Each cube is tracked from frame to frame, so a cube that stays in view
for many frames is counted only once.

Usage:
    python3 cube_counter.py                    # asks for the color, uses the Pi camera
    python3 cube_counter.py --color G          # count green cubes
    python3 cube_counter.py --color B --show   # also show the video with outlines
    python3 cube_counter.py --color R --video new.mkv   # test on a video file
"""

import argparse
import sys

import cv2
import numpy as np

# ============================== SETTINGS ==============================

# HSV ranges for each color (OpenCV: H 0-179, S 0-255, V 0-255).
# Red wraps around H = 0, so it has two ranges.
COLOR_RANGES = {
    "R": [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (179, 255, 255))],
    "G": [((35, 100, 50), (85, 255, 255))],
    "B": [((100, 120, 50), (130, 255, 255))],
}
COLOR_NAMES = {"R": "Red", "G": "Green", "B": "Blue"}
OUTLINE_BGR = {"R": (0, 0, 255), "G": (0, 255, 0), "B": (255, 0, 0)}

FRAME_SIZE = (640, 480)   # Camera resolution (width, height)
MIN_AREA_RATIO = 0.01     # Ignore blobs smaller than 1% of the frame
CONFIRM_FRAMES = 5        # A cube must be seen this many frames to be counted
MAX_MISSED_FRAMES = 10    # Forget a cube after it is missing this many frames
MATCH_MARGIN = 0.05       # Box margin when matching cubes between frames (fraction of frame width)


# ============================== DETECTION =============================

def color_mask(frame_bgr, color):
    """Returns a black/white mask of the pixels that match the color."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for low, high in COLOR_RANGES[color]:
        mask |= cv2.inRange(hsv, np.array(low), np.array(high))

    # Remove small noise and fill small holes
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def find_cubes(frame_bgr, color):
    """Returns the contours of the cubes of this color in the frame."""
    mask = color_mask(frame_bgr, color)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = MIN_AREA_RATIO * frame_bgr.shape[0] * frame_bgr.shape[1]
    return [c for c in contours if cv2.contourArea(c) >= min_area]


# ============================== TRACKING ==============================

def overlap(a, b, margin=0):
    """Overlap area of two boxes (x1, y1, x2, y2), each grown by "margin"."""
    w = min(a[2], b[2]) - max(a[0], b[0]) + 2 * margin
    h = min(a[3], b[3]) - max(a[1], b[1]) + 2 * margin
    return w * h if w > 0 and h > 0 else 0


def merge_boxes(boxes):
    """Merges overlapping boxes, so a cube split in pieces becomes one box."""
    boxes = list(boxes)
    merged = True
    while merged:
        merged = False
        for i in range(len(boxes)):
            for j in range(i + 1, len(boxes)):
                if overlap(boxes[i], boxes[j]):
                    a, b = boxes[i], boxes.pop(j)
                    boxes[i] = (min(a[0], b[0]), min(a[1], b[1]), max(a[2], b[2]), max(a[3], b[3]))
                    merged = True
                    break
            if merged:
                break
    return boxes


class CubeCounter:
    """Tracks cubes between frames and counts each one once."""

    def __init__(self, color):
        self.color = color
        self.tracks = []   # Each track: {"box", "seen", "missed", "counted"}
        self.count = 0

    def update(self, frame_bgr):
        """Processes one frame. Returns the cube contours found in it."""
        cubes = find_cubes(frame_bgr, self.color)
        margin = MATCH_MARGIN * frame_bgr.shape[1]

        boxes = []
        for c in cubes:
            x, y, w, h = cv2.boundingRect(c)
            boxes.append((x, y, x + w, y + h))
        boxes = merge_boxes(boxes)

        # Match each cube to the existing track it overlaps the most
        unmatched = list(self.tracks)
        for box in boxes:
            best, best_overlap = None, 0
            for t in unmatched:
                o = overlap(box, t["box"], margin)
                if o > best_overlap:
                    best, best_overlap = t, o
            if best is None:
                self.tracks.append({"box": box, "seen": 1, "missed": 0, "counted": False})
            else:
                unmatched.remove(best)
                best.update(box=box, seen=best["seen"] + 1, missed=0)

        # Tracks not seen in this frame
        for t in unmatched:
            t["missed"] += 1
        self.tracks = [t for t in self.tracks if t["missed"] <= MAX_MISSED_FRAMES]

        # Count tracks that have been seen long enough
        for t in self.tracks:
            if not t["counted"] and t["seen"] >= CONFIRM_FRAMES:
                t["counted"] = True
                self.count += 1
                print(f"{COLOR_NAMES[self.color]} cube #{self.count} detected")

        return cubes

    def draw(self, frame_bgr, cubes):
        """Draws outlines and the current count on the frame."""
        cv2.drawContours(frame_bgr, cubes, -1, OUTLINE_BGR[self.color], 2)
        text = f"{COLOR_NAMES[self.color]} cubes: {self.count}"
        cv2.putText(frame_bgr, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)


# ============================== CAMERA ================================

def pi_camera_frames():
    """Yields BGR frames from the Raspberry Pi camera."""
    from picamera2 import Picamera2

    cam = Picamera2()
    # "RGB888" in Picamera2 is stored as BGR, which is what OpenCV expects
    cam.configure(cam.create_video_configuration(main={"size": FRAME_SIZE, "format": "RGB888"}))
    cam.start()
    try:
        while True:
            yield cam.capture_array()
    finally:
        cam.stop()


def video_frames(path):
    """Yields BGR frames from a video file."""
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        sys.exit(f"Cannot open video: {path}")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            yield frame
    finally:
        cap.release()


# =============================== MAIN =================================

def count_cubes(color, video=None, show=False):
    """Counts cubes of "color" until the video ends or 'q' / Ctrl+C is pressed."""
    counter = CubeCounter(color)
    frames = video_frames(video) if video else pi_camera_frames()

    try:
        for frame in frames:
            cubes = counter.update(frame)
            if show:
                counter.draw(frame, cubes)
                cv2.imshow(f"{COLOR_NAMES[color]} Cube Detection", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if show:
            cv2.destroyAllWindows()

    print(f"Total {COLOR_NAMES[color]} cubes detected: {counter.count}")
    return counter.count


def main():
    parser = argparse.ArgumentParser(description="Count cubes of one color with the Pi camera.")
    parser.add_argument("--color", choices=["R", "G", "B"], type=str.upper,
                        help="Cube color to count: R, G or B")
    parser.add_argument("--video", help="Use a video file instead of the Pi camera")
    parser.add_argument("--show", action="store_true", help="Show the video with outlines")
    args = parser.parse_args()

    color = args.color
    while color not in COLOR_RANGES:
        color = input("Enter the color (R, G, or B): ").strip().upper()

    count_cubes(color, args.video, args.show)


if __name__ == "__main__":
    main()
