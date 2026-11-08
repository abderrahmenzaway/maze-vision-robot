// ============================================================
//  Gyro.ino - MPU6050 initialisation and angle reading
//  Library: MPU6050_light (rfetick), install it from the Library Manager
// ============================================================

float angle;

// Gyro PID
int16_t ma = 0;
int16_t mb = 0;
int16_t Pg;
int16_t Ig;
int16_t Dg;
int16_t lastErrorg   = 0;
int16_t errorg       = 0;
uint8_t maxspeedag   = 0;
uint8_t maxspeedbg   = 0;
int16_t basespeedag  = 0;
int16_t basespeedbg  = 0;
int motorspeedg      = 0;
int motorspeedag     = 0;
int motorspeedbg     = 0;
float Kpg = 0;
float Kdg = 0;
float Kig = 0;

// ========================= INITIALISATION ========================
// Keep the robot still and flat while this runs (about 1 s).
void initGyro(void)
{
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);  // 0 = OK
  if (status != 0) {
    Serial.println(F("MPU6050 not found, check the wiring"));
    return;
  }

  Serial.println(F("Calculating offsets, do not move the robot..."));
  delay(1000);
  mpu.calcOffsets();  // Gyro and accelerometer offsets
  Serial.println(F("Gyro ready"));
}

// ========================== ANGLE READING ========================
// Returns the current heading (yaw) in degrees, wrapped to -180 .. 180.
// MPU6050_light integrates the gyro on each update(), so call this often
// (every loop iteration while moving).
float getAngle(void)
{
  mpu.update();
  float a = fmod(mpu.getAngleZ() + 180.0, 360.0);
  if (a < 0) a += 360.0;
  return a - 180.0;
}

// ========================= GYRO MOVES ============================

// Drive straight holding the heading "Setpoint" (PID, no angle wrap)
void gyroDriveStraightPID(float Setpoint, int16_t vitesse)
{
  maxspeedag  = 255;
  maxspeedbg  = 255;
  basespeedag = vitesse;
  basespeedbg = vitesse;
  Kpg = 9;  // Tune first
  Kdg = 5;  // Tune second
  Kig = 0;

  angle  = getAngle();
  errorg = Setpoint - angle;
  Pg = errorg;
  Ig = Ig + errorg;
  Dg = errorg - lastErrorg;

  motorspeedg  = Pg * Kpg + Ig * Kig + Dg * Kdg;
  motorspeedag = basespeedag + motorspeedg;
  motorspeedbg = basespeedbg - motorspeedg;

  if (motorspeedag > maxspeedag) motorspeedag = maxspeedag;
  if (motorspeedbg > maxspeedbg) motorspeedbg = maxspeedbg;

  setMotorSpeeds(motorspeedag, motorspeedbg);
  lastErrorg = errorg;
}

// Drive forward, correcting slightly toward heading x
void gyroForward(int16_t x, int16_t vitesse)
{
  if (getAngle() > x) {
    setMotorSpeeds(vitesse, vitesse + 5);
  } else {
    setMotorSpeeds(vitesse + 5, vitesse);
  }
}

// Drive backward, correcting slightly toward heading x
void gyroBackward(int16_t x, int16_t vitesse)
{
  if (getAngle() > x) {
    setMotorSpeeds(-vitesse - 10, -vitesse);
  } else {
    setMotorSpeeds(-vitesse, -vitesse - 10);
  }
}
