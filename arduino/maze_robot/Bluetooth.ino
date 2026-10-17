// ============================================================
//  Bluetooth.ino - remote control over an HC-05 / HC-06 module
// ============================================================
//
//  Wiring (Arduino Mega, Serial2):
//    HC-05 TX -> pin 17 (RX2)
//    HC-05 RX -> pin 16 (TX2)   (use a voltage divider, module RX is 3.3V)
//  Serial1 (pins 18/19) is not used because ECHO/TRIG are on those pins.
//
//  Commands:
//    'F' -> forward     'B' -> backward
//    'R' -> right       'L' -> left
//    'D' -> dance (turn right for 300 ms)
//    'S' -> stop

#define BT_SERIAL     Serial2
#define BT_BAUDRATE   9600
#define BT_SPEED      150   // Motor speed for Bluetooth moves (0..255)
#define DANCE_TIME_MS 300

void initBluetooth(void)
{
  BT_SERIAL.begin(BT_BAUDRATE);
}

void handleVoiceCommand(void)
{
  if (!BT_SERIAL.available()) return;

  char cmd = BT_SERIAL.read();

  switch (cmd) {
    case 'F':  // Forward
      setMotorSpeeds(BT_SPEED, BT_SPEED);
      break;
    case 'B':  // Backward
      setMotorSpeeds(-BT_SPEED, -BT_SPEED);
      break;
    case 'R':  // Turn right
      setMotorSpeeds(BT_SPEED, -BT_SPEED);
      break;
    case 'L':  // Turn left
      setMotorSpeeds(-BT_SPEED, BT_SPEED);
      break;
    case 'D':  // Dance: turn right for 300 ms
      setMotorSpeeds(BT_SPEED, -BT_SPEED);
      delay(DANCE_TIME_MS);
      stopMotors();
      break;
    case 'S':  // Stop
      stopMotors();
      break;
    default:   // Ignore anything else (e.g. '\r', '\n')
      break;
  }
}
