"""
pi_link.py - 2-bit GPIO link from the Raspberry Pi to the Arduino Mega.

The Pi sends a code from 0 to 3 to the Arduino on two GPIO lines:

    code   BIT1 (GPIO 27)   BIT0 (GPIO 17)
     0          LOW              LOW
     1          LOW              HIGH
     2          HIGH             LOW
     3          HIGH             HIGH

The code is a level, not a message: the Pi keeps it on the lines until it
sends another one, and the Arduino reads it whenever it needs it. What each
code means depends on the mission (for example a colour, a number of cubes
or a direction), so the meaning is defined in the mission program, not here.

Usage as a module:
    from pi_link import PiLink
    link = PiLink()
    link.send(2)

Usage from the command line, to check the wiring:
    python3 pi_link.py 2        # put code 2 on the lines
    python3 pi_link.py --cycle  # show 0, 1, 2, 3 in a loop (1 s each)
"""
import argparse
import time

BIT0_GPIO = 17  # least significant bit
BIT1_GPIO = 27  # most significant bit

try:
    from gpiozero import DigitalOutputDevice
except ImportError:  # running on a PC: print instead of driving pins
    DigitalOutputDevice = None


class _FakePin:
    def __init__(self, gpio):
        self.gpio = gpio
        self.value = 0

    def on(self):
        self.value = 1

    def off(self):
        self.value = 0

    def close(self):
        pass


class PiLink:
    """Drives the two link lines. Both start LOW (code 0)."""

    def __init__(self, bit0=BIT0_GPIO, bit1=BIT1_GPIO):
        make = DigitalOutputDevice if DigitalOutputDevice else _FakePin
        self.bit0 = make(bit0)
        self.bit1 = make(bit1)
        self.code = None
        self.send(0)

    def send(self, code):
        if code not in (0, 1, 2, 3):
            raise ValueError(f"code must be 0, 1, 2 or 3, got {code!r}")
        self.bit1.on() if code & 0b10 else self.bit1.off()
        self.bit0.on() if code & 0b01 else self.bit0.off()
        self.code = code
        if DigitalOutputDevice is None:
            print(f"[pi_link] code {code} -> BIT1={code >> 1 & 1} BIT0={code & 1}")

    def close(self):
        self.send(0)
        self.bit0.close()
        self.bit1.close()


def main():
    parser = argparse.ArgumentParser(description="Put a 2-bit code (0 to 3) on the Pi to Arduino link.")
    parser.add_argument("code", nargs="?", type=int, choices=[0, 1, 2, 3])
    parser.add_argument("--cycle", action="store_true", help="show 0, 1, 2, 3 in a loop")
    args = parser.parse_args()

    link = PiLink()
    try:
        if args.cycle:
            while True:
                for code in range(4):
                    link.send(code)
                    print(f"code {code}")
                    time.sleep(1)
        elif args.code is not None:
            link.send(args.code)
            print(f"code {args.code} on the lines, Ctrl+C to stop")
            while True:
                time.sleep(1)
        else:
            parser.print_help()
    except KeyboardInterrupt:
        pass
    finally:
        link.close()


if __name__ == "__main__":
    main()
