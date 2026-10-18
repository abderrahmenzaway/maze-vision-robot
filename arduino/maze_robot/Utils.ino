// ============================================================
//  Utils.ino - motors, RGB LED, buzzer, sensors, OLED display
// ============================================================

// ============================= MOTORS ==============================
// Speeds from -255 to 255. 0 = brake.
void setMotorSpeeds(int16_t left, int16_t right)
{
  if (left == 0) MotorL.brake();
  else           MotorL.drive(left);

  if (right == 0) MotorR.brake();
  else            MotorR.drive(right);
}

void stopMotors(void)
{
  MotorR.brake();
  MotorL.brake();
}

// ============================ RGB LED ==============================
// setLed(1,0,0) red   | setLed(0,1,0) green   | setLed(0,0,1) blue
// setLed(0,1,1) cyan  | setLed(1,0,1) magenta | setLed(1,1,0) yellow | setLed(1,1,1) white
void setLed(uint8_t red_value, uint8_t green_value, uint8_t blue_value)
{
  digitalWrite(LED_R, red_value);
  digitalWrite(LED_G, green_value);
  digitalWrite(LED_B, blue_value);
}

// Blink red LED + buzzer "nb" times, "timing" ms on / off
void blinkAlert(uint8_t nb, uint16_t timing)
{
  for (uint8_t i = 0; i < nb; i++) {
    setLed(1, 0, 0);
    digitalWrite(BUZZER, HIGH);
    delay(timing);

    setLed(0, 0, 0);
    digitalWrite(BUZZER, LOW);
    delay(timing);
  }
}

// ============================ SENSORS ==============================
uint8_t readProximityLeft(void)
{
  return digitalRead(PROX_L);
}

uint8_t readProximityRight(void)
{
  return digitalRead(PROX_R);
}

// Ultrasonic distance in cm
float readDistanceCm(void)
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  float duration_us = pulseIn(ECHO, HIGH);
  return 0.017 * duration_us;
}

// ============================ DISPLAY ==============================
// Shows the heading, the front distance and the two side wall sensors
void showSensorScreen(void)
{
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.print("Angle: ");
  display.print(getAngle());

  display.setCursor(0, 16);
  display.print("Front: ");
  display.print(readDistanceCm());
  display.print(" cm");

  display.setCursor(0, 32);
  display.print("Left IR: ");
  display.print(readProximityLeft());
  display.print("  Right IR: ");
  display.print(readProximityRight());

  display.setCursor(0, 48);
  display.print("Pi code: ");
  display.print(readPiCode());

  display.display();
}

void showText(String message)
{
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(20, 30);
  display.print(message);
  display.display();
}
