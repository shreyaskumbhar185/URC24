#include <Ps3Controller.h>

// Motor pins for L298N Motor Driver
#define MOTOR1_IN1 5  // Motor 1 forward pin
#define MOTOR1_IN2 18 // Motor 1 backward pin
#define MOTOR2_IN1 19 // Motor 2 forward pin
#define MOTOR2_IN2 21 // Motor 2 backward pin

// Enable pins for motor speed control (PWM)
#define MOTOR1_EN 4  // Motor 1 speed control
#define MOTOR2_EN 12 // Motor 2 speed control

// Motor speed value (fixed for D-pad control)
int motorSpeed = 255; // Value from 0-255 for PWM control

// Callback function to process PS3 controller inputs
void notify() {
  // D-pad control for motor movement
  if (Ps3.data.button.up) {
    controlMotor(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_EN, motorSpeed);  // Motor 1 forward
    controlMotor(MOTOR2_IN1, MOTOR2_IN2, MOTOR2_EN, motorSpeed);  // Motor 2 forward
  } else if (Ps3.data.button.down) {
    controlMotor(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_EN, -motorSpeed); // Motor 1 backward
    controlMotor(MOTOR2_IN1, MOTOR2_IN2, MOTOR2_EN, -motorSpeed); // Motor 2 backward
  } else if (Ps3.data.button.left) {
    controlMotor(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_EN, -motorSpeed); // Motor 1 backward
    controlMotor(MOTOR2_IN1, MOTOR2_IN2, MOTOR2_EN, motorSpeed);  // Motor 2 forward
  } else if (Ps3.data.button.right) {
    controlMotor(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_EN, motorSpeed);  // Motor 1 forward
    controlMotor(MOTOR2_IN1, MOTOR2_IN2, MOTOR2_EN, -motorSpeed); // Motor 2 backward
  } else {
    // Stop both motors if no D-pad input
    controlMotor(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_EN, 0); 
    controlMotor(MOTOR2_IN1, MOTOR2_IN2, MOTOR2_EN, 0);
  }

  // Debugging output for D-pad buttons
  Serial.print("Up: "); Serial.print(Ps3.data.button.up);
  Serial.print(", Down: "); Serial.print(Ps3.data.button.down);
  Serial.print(", Left: "); Serial.print(Ps3.data.button.left);
  Serial.print(", Right: "); Serial.println(Ps3.data.button.right);
}

// Function to control individual motors
void controlMotor(int motorPin1, int motorPin2, int motorEN, int speed) {
  if (speed > 0) { // Move forward
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    analogWrite(motorEN, speed);
  } else if (speed < 0) { // Move backward
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    analogWrite(motorEN, abs(speed));
  } else { // Stop the motor
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    analogWrite(motorEN, 0);
  }
}

// On Connection function
void onConnect() {
  Serial.println("Controller Connected.");
}

void setup() {
  Serial.begin(115200);

  // Initialize motor pins as outputs
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);

  // Initialize PWM pins for speed control
  pinMode(MOTOR1_EN, OUTPUT);
  pinMode(MOTOR2_EN, OUTPUT);

  // Attach the notify function to the PS3 controller
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  
  // Start PS3 controller communication with a given MAC address
  Ps3.begin("00:00:00:00:00:0a");

  Serial.println("Ready.");
}

void loop() {
  if (!Ps3.isConnected()) {
    return;
  }
  delay(100);
}
