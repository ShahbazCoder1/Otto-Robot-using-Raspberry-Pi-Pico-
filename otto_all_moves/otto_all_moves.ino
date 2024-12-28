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
  setServo(RightFoot, 80);
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
  homePosition();
}

void walk() {
  // Basic walking sequence
  // Step 1: Lift left foot
  setServo(LeftFoot, 70);
  delay(200);
  
  // Step 2: Move left leg forward
  setServo(LeftLeg, 120);
  delay(200);
  
  // Step 3: Lower left foot
  setServo(LeftFoot, 90);
  delay(200);
  
  // Step 4: Lift right foot
  setServo(RightFoot, 70);
  delay(200);
  
  // Step 5: Move right leg forward
  setServo(RightLeg, 120);
  delay(200);
  
  // Step 6: Lower right foot
  setServo(RightFoot, 90);
  delay(200);
  
  homePosition();
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
  delay(500);
  hello();
  delay(5000);
  for (int i = 0; i < 5; i++) {
    walk();
  }
  delay(5000);
  dance();
  delay(5000);
}
