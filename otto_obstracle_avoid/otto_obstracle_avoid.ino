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

// Define ultrasonic sensor pins
#define TRIG_PIN 7
#define ECHO_PIN 6

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

  // Initialize ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
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

long measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2; // Convert to cm
  return distance;
}

void walkForward() {
  // Basic walking sequence
  setServo(LeftFoot, 70);
  delay(200);
  setServo(LeftLeg, 120);
  delay(200);
  setServo(LeftFoot, 90);
  delay(200);

  setServo(RightFoot, 70);
  delay(200);
  setServo(RightLeg, 120);
  delay(200);
  setServo(RightFoot, 90);
  delay(200);
}

void walkBackward() {
  // Basic backward walking sequence
  setServo(RightFoot, 110);
  delay(200);
  setServo(RightLeg, 60);
  delay(200);
  setServo(RightFoot, 90);
  delay(200);

  setServo(LeftFoot, 110);
  delay(200);
  setServo(LeftLeg, 60);
  delay(200);
  setServo(LeftFoot, 90);
  delay(200);
}

void turnLeft() {
  setServo(LeftLeg, 120);
  setServo(RightLeg, 60);
  delay(500);
  homePosition();
}

void turnRight() {
  setServo(LeftLeg, 60);
  setServo(RightLeg, 120);
  delay(500);
  homePosition();
}

void avoidObstacle() {
  long distance = measureDistance();
  if (distance < 20) {
    Serial.println("Obstacle detected! Turning...");
    walkBackward();
    turnRight();
  } else {
    walkForward();
  }
}

void loop() {
  avoidObstacle();
  delay(100);
}
