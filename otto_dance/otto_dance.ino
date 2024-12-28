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

void dance() {
  // Dance move 1: Side-to-side sway
  for (int i = 0; i < 3; i++) {
    setServo(LeftLeg, 80);
    setServo(RightLeg, 100);
    delay(300);
    setServo(LeftLeg, 100);
    setServo(RightLeg, 80);
    delay(300);
    homePosition();
  }

  // Foot tapping
  for (int i = 0; i < 3; i++) {
    setServo(LeftFoot, 60);
    delay(200);
    setServo(LeftFoot, 90);
    delay(200);
    setServo(RightFoot, 60);
    delay(200);
    setServo(RightFoot, 90);
    delay(200);
    homePosition();
  }

  // Twisting legs
  for (int i = 0; i < 3; i++) {
    setServo(LeftLeg, 70);
    setServo(RightLeg, 110);
    delay(300);
    setServo(LeftLeg, 110);
    setServo(RightLeg, 70);
    delay(300);
    homePosition();
  }

  // Wave legs up and down
  for (int i = 0; i < 3; i++) {
    setServo(LeftLeg, 60);
    setServo(RightLeg, 120);
    delay(300);
    setServo(LeftFoot, 60);
    setServo(RightFoot, 120);
    delay(300);
    homePosition();
  }

  // Sideways shuffle
  for (int i = 0; i < 3; i++) {
    setServo(LeftLeg, 70);
    setServo(RightLeg, 110);
    setServo(LeftFoot, 80);
    setServo(RightFoot, 100);
    delay(200);
    
    setServo(LeftLeg, 110);
    setServo(RightLeg, 70);
    setServo(LeftFoot, 100);
    setServo(RightFoot, 80);
    delay(200);
    homePosition();
  }

  // Return to home position
  homePosition();
}

void loop() {
  homePosition();
  dance(); 
  delay(1000); 
}
