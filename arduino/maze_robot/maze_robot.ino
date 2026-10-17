/*
 * maze_robot - Autonomous maze robot, Arduino side (Arduino Mega 2560 Pro)
 *
 * Sketch files (the Arduino IDE merges them in this order):
 *   maze_robot.ino    -> includes, pin config, global objects, setup() and loop()
 *   Bluetooth.ino     -> Bluetooth remote control (handleVoiceCommand)
 *   Gyro.ino          -> MPU6050 init, angle reading, gyro-based moves
 *   Line_Follower.ino -> QTR calibration, speed presets, PID line following
 *   Utils.ino         -> motors, LED, buzzer, sensors, OLED display
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_TB6612.h>
#include <QTRSensors.h>
#include <MPU6050_6Axis_MotionApps612.h>

// ============================== MPU6050 ==============================
// Default I2C address is 0x68 (AD0 low). Use MPU6050 mpu(0x69) if AD0 is high.
MPU6050 mpu;

// ======================== TB6612FNG MOTOR DRIVER ======================
#define AIN1    2
#define AIN2    4
#define BIN1    12
#define BIN2    10
#define PWMA    6   // Right motor PWM
#define PWMB    8   // Left motor PWM
#define STBY    -1  // Standby pin (not used)
#define OFFSETL -1
#define OFFSETR 1

Motor MotorL = Motor(BIN1, BIN2, PWMB, OFFSETL, STBY);
Motor MotorR = Motor(AIN1, AIN2, PWMA, OFFSETR, STBY);

// ============================ OLED DISPLAY ============================
#define SCREEN_WIDTH  128  // OLED width, in pixels
#define SCREEN_HEIGHT 64   // OLED height, in pixels
#define OLED_RESET    -1   // Reset pin (-1 = shared with Arduino reset)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// =============================== DEVICES ==============================
#define LED_R   39
#define LED_G   37
#define LED_B   35

#define BUZZER  41
#define BTN     33

#define ECHO    18
#define TRIG    19

#define PROX_L  22
#define PROX_R  23

// Extra digital outputs used by the sensor board
#define OUT_38  38
#define OUT_36  36
#define OUT_34  34
#define OUT_32  32

// ============================= QTR SENSORS ============================
#define SENSORCOUNT  16
#define PIN_SENSOR_0 A14

uint16_t SensorValues[SENSORCOUNT];

const uint8_t Analog_Pins[SENSORCOUNT] = {
  PIN_SENSOR_0, PIN_SENSOR_0, PIN_SENSOR_0, PIN_SENSOR_0,
  PIN_SENSOR_0, PIN_SENSOR_0, PIN_SENSOR_0, PIN_SENSOR_0,
  PIN_SENSOR_0, PIN_SENSOR_0, PIN_SENSOR_0, PIN_SENSOR_0,
  PIN_SENSOR_0, PIN_SENSOR_0, PIN_SENSOR_0, PIN_SENSOR_0
};

QTRSensors qtr;

// ========================== LINE FOLLOWER PID =========================
uint16_t position;
int16_t P;
int16_t I;
int16_t D;
int16_t lastError   = 0;
int16_t error       = 0;
int16_t motorspeed  = 0;
int16_t motorspeedl = 0;  // Left
int16_t motorspeedr = 0;  // Right
int16_t MAXSPEEDR   = 0;  // Max speed, right motor
int16_t MAXSPEEDL   = 0;  // Max speed, left motor
int16_t BASESPEEDR  = 0;  // Base speed, right motor
int16_t BASESPEEDL  = 0;  // Base speed, left motor
float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;

// =============================== SETUP ================================
void setup(void)
{
  Serial.begin(9600);
  Wire.begin();
  initBluetooth();

  pinMode(BUZZER, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(BTN, INPUT);
  pinMode(PROX_R, INPUT);
  pinMode(PROX_L, INPUT);
  pinMode(PIN_SENSOR_0, INPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(OUT_38, OUTPUT);
  pinMode(OUT_36, OUTPUT);
  pinMode(OUT_34, OUTPUT);
  pinMode(OUT_32, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins(Analog_Pins, SENSORCOUNT);

  // Try OLED at 0x3C, then 0x3D
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 not found at 0x3C"));
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
      Serial.println(F("No OLED display found"));
      while (1);  // Stop here
    }
  }

  startupSequence();
}

// ================================ LOOP ================================
void loop(void)
{
  handleVoiceCommand();
}

// ========================== STARTUP SEQUENCE ==========================
static void startupSequence(void)
{
  // Step 1: gyro calibration
  showText("Calibrate gyro?");
  while (digitalRead(BTN) == 0);  // Wait for button press
  blinkAlert(2, 100);
  setLed(0, 1, 0);                   // Green LED: gyro calibration running
  initGyro();
  setLed(0, 0, 0);
  showText("Gyro calibrated");
  blinkAlert(2, 100);

  // Step 2: QTR sensor calibration
  showText("Calibrate QTR?");
  while (digitalRead(BTN) == 0);  // Wait for button press
  calibrateSensors();
  setLed(0, 0, 0);
  blinkAlert(2, 100);
  showText("QTR calibrated");
  delay(500);

  // Show gyro angle and sensor bars until the button is pressed
  while (digitalRead(BTN) == 0) {
    showSensorScreen();
  }
}
