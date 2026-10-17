// ============================================================
//  Line_Follower.ino - QTR calibration, speed presets, PID line follow
// ============================================================

// ========================== SPEED PRESETS ==========================
void maxSpeedConfig(void)
{
  Kp = 0.07;
  Kd = 0.2;
  MAXSPEEDR  = 200;
  MAXSPEEDL  = 200;
  BASESPEEDR = MAXSPEEDR - 20;
  BASESPEEDL = MAXSPEEDL - 20;
}

void mediumSpeedConfig(void)
{
  Kp = 0.06;
  Kd = 0.25;
  MAXSPEEDR  = 150;
  MAXSPEEDL  = 150;
  BASESPEEDR = MAXSPEEDR - 20;
  BASESPEEDL = MAXSPEEDL - 20;
}

void minSpeedConfig(void)
{
  Kp = 0.05;
  Kd = 0.15;
  MAXSPEEDR  = 100;
  MAXSPEEDL  = 100;
  BASESPEEDR = MAXSPEEDR - 20;
  BASESPEEDL = MAXSPEEDL - 20;
}

// ========================= PID LINE FOLLOW =========================
// mode: 'B' = black line on white, 'W' = white line on black
void followLine(char mode)
{
  if (mode == 'B') {
    position = qtr.readLineBlack(SensorValues);
  } else if (mode == 'W') {
    position = qtr.readLineWhite(SensorValues);
  }

  // Error relative to the center of the sensor bar
  error = (((SENSORCOUNT - 1) * 1000) / 2) - position;
  P = error;
  I += error;
  D = error - lastError;

  motorspeed  = (P * Kp) + (I * Ki) + (D * Kd);
  motorspeedr = BASESPEEDR + motorspeed;
  motorspeedl = BASESPEEDL - motorspeed;

  motorspeedl = constrain(motorspeedl, -MAXSPEEDL, MAXSPEEDL);
  motorspeedr = constrain(motorspeedr, -MAXSPEEDR, MAXSPEEDR);

  setMotorSpeeds(motorspeedl, motorspeedr);
  lastError = error;
}

// =========================== CALIBRATION ===========================
void calibrateSensors(void)
{
  setLed(0, 0, 1);  // Blue LED: calibration running
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
  }
  setLed(0, 1, 0);  // Green LED: done
  delay(1500);

  // Print min / max calibration values
  for (uint8_t i = 0; i < SENSORCOUNT; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
  for (uint8_t i = 0; i < SENSORCOUNT; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(500);
}
