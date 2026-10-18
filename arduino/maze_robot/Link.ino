/*
 * Link.ino - 2-bit GPIO link from the Raspberry Pi (Arduino side)
 *
 * The Pi puts a code from 0 to 3 on two wires:
 *
 *   code   BIT1 (pin PI_BIT1)   BIT0 (pin PI_BIT0)
 *    0          LOW                  LOW
 *    1          LOW                  HIGH
 *    2          HIGH                 LOW
 *    3          HIGH                 HIGH
 *
 * The code is a level held by the Pi, so the Arduino can read it at any time.
 * The Pi changes the two bits one after the other, so a read taken during a
 * change could see a wrong code for a few microseconds. readPiCode() avoids
 * this by reading twice, 2 ms apart, until both reads agree.
 *
 * What each code means depends on the mission program.
 */

void initPiLink(void)
{
  pinMode(PI_BIT0, INPUT);
  pinMode(PI_BIT1, INPUT);
}

static uint8_t readPiLinesOnce(void)
{
  return (uint8_t)((digitalRead(PI_BIT1) << 1) | digitalRead(PI_BIT0));
}

// Returns the current code (0 to 3), once it is stable.
uint8_t readPiCode(void)
{
  uint8_t first = readPiLinesOnce();
  for (uint8_t tries = 0; tries < 10; tries++) {
    delay(2);
    uint8_t second = readPiLinesOnce();
    if (second == first) {
      return second;
    }
    first = second;
  }
  return first;  // lines kept changing: return the last value read
}

// Waits until the Pi sends `code`. Returns true if it arrived before
// `timeoutMs` (0 = wait forever), false on timeout.
bool waitForPiCode(uint8_t code, unsigned long timeoutMs)
{
  unsigned long start = millis();
  while (readPiCode() != code) {
    if (timeoutMs != 0 && millis() - start >= timeoutMs) {
      return false;
    }
  }
  return true;
}
