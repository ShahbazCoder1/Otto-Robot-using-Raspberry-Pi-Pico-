#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50
#define SERVOMIN 150
#define SERVOMAX 600

// Define servo channels on PCA9685
#define LeftLeg 0
#define RightLeg 1
#define LeftFoot 2
#define RightFoot 3

// Calibration values
int YL = 0;  // Left Leg trim
int YR = 0;  // Right Leg trim
int RL = 0;  // Left Foot trim
int RR = 0;  // Right Foot trim

void setup() {
  Serial.begin(9600);

  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
}

int angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void setServo(uint8_t servoNum, int angle) {
  pwm.setPWM(servoNum, 0, angleToPulse(angle));
}

void homePosition() {
  setServo(LeftLeg, 90);
  setServo(RightLeg, 90);
  setServo(LeftFoot, 90);
  setServo(RightFoot, 90);
}

void hello() {
  setServo(LeftFoot, 60);
  for (int i = 0; i < 3; i++) {
    setServo(RightLeg, 60);
    delay(300);
    setServo(RightLeg, 90);
    delay(300);
    setServo(RightFoot, 60);
    delay(300);
    setServo(RightFoot, 90);
    delay(300);
  }
}

void loop() {
  homePosition();
  delay(500);
  hello();
  delay(2000);
}
