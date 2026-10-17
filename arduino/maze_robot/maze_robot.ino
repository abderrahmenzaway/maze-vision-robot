/*
 * maze_robot - Autonomous maze robot, Arduino side (Arduino Mega 2560 Pro)
 *
 * Sketch files (the Arduino IDE merges them in this order):
 *   maze_robot.ino    -> includes, pin config, global objects, setup() and loop()
 *   Gyro.ino          -> MPU6050 init, angle reading, gyro-based moves
 *   Utils.ino         -> motors, wall sensors, LED, buzzer, OLED display
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_TB6612.h>
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

// =============================== SETUP ================================
void setup(void)
{
  Serial.begin(9600);
  Wire.begin();

  pinMode(BUZZER, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(BTN, INPUT);
  pinMode(PROX_R, INPUT);
  pinMode(PROX_L, INPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

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
  // Sensor check: heading, front distance and side walls on the OLED.
  // The maze navigation sequence is called from here.
  showSensorScreen();
}

// ========================== STARTUP SEQUENCE ==========================
static void startupSequence(void)
{
  // Gyro calibration
  showText("Calibrate gyro?");
  while (digitalRead(BTN) == 0);  // Wait for button press
  blinkAlert(2, 100);
  setLed(0, 1, 0);                   // Green LED: gyro calibration running
  initGyro();
  setLed(0, 0, 0);
  showText("Gyro calibrated");
  blinkAlert(2, 100);

  // Show the sensors until the button is pressed
  while (digitalRead(BTN) == 0) {
    showSensorScreen();
  }
}
