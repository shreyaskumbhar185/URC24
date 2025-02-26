#include <Ps3Controller.h>

// ========================
// Pin Definitions
// ========================

// First L298N Module (Motor 1 & Motor 2)
#define ENA1_PIN 22   // PWM pin for Motor 1
#define IN1_1_PIN 18  // Direction pin 1 for Motor 1
#define IN2_1_PIN 5   // Direction pin 2 for Motor 1
#define ENB1_PIN 23   // PWM pin for Motor 2
#define IN3_1_PIN 19  // Direction pin 1 for Motor 2
#define IN4_1_PIN 21  // Direction pin 2 for Motor 2

// Second L298N Module (Motor 3 & Motor 4)
#define ENA2_PIN 25   // PWM pin for Motor 3
#define IN1_2_PIN 26  // Direction pin 1 for Motor 3
#define IN2_2_PIN 27  // Direction pin 2 for Motor 3
#define ENB2_PIN 32   // PWM pin for Motor 4
#define IN3_2_PIN 33  // Direction pin 1 for Motor 4
#define IN4_2_PIN 14  // Direction pin 2 for Motor 4

// ========================
// PWM Configuration
// ========================
const int motorFreq = 1000;       // PWM frequency in Hz
const int motorResolution = 8;    // PWM resolution (8-bit: 0-255)

// Define PWM channels for each motor
const int motor1Channel = 0;      // PWM channel for Motor 1 (ENA1)
const int motor2Channel = 1;      // PWM channel for Motor 2 (ENB1)
const int motor3Channel = 2;      // PWM channel for Motor 3 (ENA2)
const int motor4Channel = 3;      // PWM channel for Motor 4 (ENB2)

// ========================
// Motor Speed Variables
// ========================
int motor1Speed = 0;
int motor2Speed = 0;
int motor3Speed = 0;
int motor4Speed = 0;

// ========================
// Direction Variables
// ========================
// For each motor, true = forward, false = reverse
bool motor1Dir = true;
bool motor2Dir = true;
bool motor3Dir = true;
bool motor4Dir = true;

// ========================
// D-Pad Button States
// ========================
bool dpadUp = false;
bool dpadDown = false;
bool dpadLeft = false;
bool dpadRight = false;

// ========================
// Callback Functions
// ========================

// Callback Function when PS3 controller sends data
void notify() {
  // Retrieve D-pad button states
  dpadUp = Ps3.data.button.up;
  dpadDown = Ps3.data.button.down;
  dpadLeft = Ps3.data.button.left;
  dpadRight = Ps3.data.button.right;

  // Initialize motor speeds to zero
  motor1Speed = motor2Speed = motor3Speed = motor4Speed = 0;

  // Initialize motor directions to forward
  motor1Dir = motor2Dir = motor3Dir = motor4Dir = true;

  // Define a base speed for movement
  const int baseSpeed = 200; // Adjust as needed (0-255)

  // Determine movement based on D-pad input
  if (dpadUp) {
    // Move Forward: All motors forward
    motor1Speed = motor2Speed = motor3Speed = motor4Speed = baseSpeed;
  }
  if (dpadDown) {
    // Move Reverse: All motors reverse
    motor1Speed = motor2Speed = motor3Speed = motor4Speed = baseSpeed;
    motor1Dir = motor2Dir = motor3Dir = motor4Dir = false;
  }
  if (dpadLeft) {
    // Turn Left: Left motors reverse, Right motors forward
    motor1Speed = motor3Speed = baseSpeed;
    motor2Speed = motor4Speed = baseSpeed;
    motor1Dir = motor3Dir = false;
    motor2Dir = motor4Dir = true;
  }
  if (dpadRight) {
    // Turn Right: Left motors forward, Right motors reverse
    motor1Speed = motor3Speed = baseSpeed;
    motor2Speed = motor4Speed = baseSpeed;
    motor1Dir = motor3Dir = true;
    motor2Dir = motor4Dir = false;
  }

  // If multiple directions are pressed (e.g., Up + Left), prioritize Forward and Turning
  // This logic can be customized as needed
  if (dpadUp && dpadLeft) {
    // Move Forward and Turn Left
    motor1Speed = motor3Speed = baseSpeed;
    motor2Speed = motor4Speed = baseSpeed;
    motor1Dir = motor3Dir = false; // Reverse left motors
    motor2Dir = motor4Dir = true;  // Forward right motors
  }
  if (dpadUp && dpadRight) {
    // Move Forward and Turn Right
    motor1Speed = motor3Speed = baseSpeed;
    motor2Speed = motor4Speed = baseSpeed;
    motor1Dir = motor3Dir = true;  // Forward left motors
    motor2Dir = motor4Dir = false; // Reverse right motors
  }
  if (dpadDown && dpadLeft) {
    // Move Reverse and Turn Left
    motor1Speed = motor3Speed = baseSpeed;
    motor2Speed = motor4Speed = baseSpeed;
    motor1Dir = motor3Dir = true;  // Forward left motors
    motor2Dir = motor4Dir = false; // Reverse right motors
  }
  if (dpadDown && dpadRight) {
    // Move Reverse and Turn Right
    motor1Speed = motor3Speed = baseSpeed;
    motor2Speed = motor4Speed = baseSpeed;
    motor1Dir = motor3Dir = false; // Reverse left motors
    motor2Dir = motor4Dir = true;  // Forward right motors
  }

  // Drive the motors with calculated speeds and directions
  driveMotors(motor1Speed, motor2Speed, motor3Speed, motor4Speed,
             motor1Dir, motor2Dir, motor3Dir, motor4Dir);

  // Debugging: Print D-pad states and motor configurations
  Serial.print("D-Pad - Up: ");
  Serial.print(dpadUp);
  Serial.print(" | Down: ");
  Serial.print(dpadDown);
  Serial.print(" | Left: ");
  Serial.print(dpadLeft);
  Serial.print(" | Right: ");
  Serial.println(dpadRight);

  Serial.print("Motor1: ");
  Serial.print(motor1Speed);
  Serial.print(" (");
  Serial.print(motor1Dir ? "F" : "R");
  Serial.print(") | Motor2: ");
  Serial.print(motor2Speed);
  Serial.print(" (");
  Serial.print(motor2Dir ? "F" : "R");
  Serial.print(") | Motor3: ");
  Serial.print(motor3Speed);
  Serial.print(" (");
  Serial.print(motor3Dir ? "F" : "R");
  Serial.print(") | Motor4: ");
  Serial.print(motor4Speed);
  Serial.print(" (");
  Serial.print(motor4Dir ? "F" : "R");
  Serial.println(")");
}

// Callback Function when PS3 controller connects
void onConnect() {
  Serial.println("PS3 Controller Connected.");
}

// ========================
// Motor Control Functions
// ========================

// Function to drive all four motors
void driveMotors(int m1, int m2, int m3, int m4,
                bool dir1, bool dir2, bool dir3, bool dir4) {
  // Motor 1 Control
  if (!dir1) {
    digitalWrite(IN1_1_PIN, HIGH);
    digitalWrite(IN2_1_PIN, LOW);
  }
  else {
    digitalWrite(IN1_1_PIN, LOW);
    digitalWrite(IN2_1_PIN, HIGH);
  }
  ledcWrite(motor1Channel, m1);

  // Motor 2 Control
  if (!dir2) {
    digitalWrite(IN3_1_PIN, HIGH);
    digitalWrite(IN4_1_PIN, LOW);
  }
  else {
    digitalWrite(IN3_1_PIN, LOW);
    digitalWrite(IN4_1_PIN, HIGH);
  }
  ledcWrite(motor2Channel, m2);

  // Motor 3 Control
  if (!dir3) {
    digitalWrite(IN1_2_PIN, HIGH);
    digitalWrite(IN2_2_PIN, LOW);
  }
  else {
    digitalWrite(IN1_2_PIN, LOW);
    digitalWrite(IN2_2_PIN, HIGH);
  }
  ledcWrite(motor3Channel, m3);

  // Motor 4 Control
  if (!dir4) {
    digitalWrite(IN3_2_PIN, HIGH);
    digitalWrite(IN4_2_PIN, LOW);
  }
  else {
    digitalWrite(IN3_2_PIN, LOW);
    digitalWrite(IN4_2_PIN, HIGH);
  }
  ledcWrite(motor4Channel, m4);
}

// ========================
// Setup Function
// ========================
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  delay(1000); // Allow time for Serial Monitor to initialize

  // Initialize motor control pins as OUTPUT
  // First L298N Module
  pinMode(ENA1_PIN, OUTPUT);
  pinMode(IN1_1_PIN, OUTPUT);
  pinMode(IN2_1_PIN, OUTPUT);
  pinMode(ENB1_PIN, OUTPUT);
  pinMode(IN3_1_PIN, OUTPUT);
  pinMode(IN4_1_PIN, OUTPUT);

  // Second L298N Module
  pinMode(ENA2_PIN, OUTPUT);
  pinMode(IN1_2_PIN, OUTPUT);
  pinMode(IN2_2_PIN, OUTPUT);
  pinMode(ENB2_PIN, OUTPUT);
  pinMode(IN3_2_PIN, OUTPUT);
  pinMode(IN4_2_PIN, OUTPUT);

  // Setup PWM channels
  ledcSetup(motor1Channel, motorFreq, motorResolution);
  ledcSetup(motor2Channel, motorFreq, motorResolution);
  ledcSetup(motor3Channel, motorFreq, motorResolution);
  ledcSetup(motor4Channel, motorFreq, motorResolution);

  // Attach PWM channels to respective GPIO pins
  ledcAttachPin(ENA1_PIN, motor1Channel);
  ledcAttachPin(ENB1_PIN, motor2Channel);
  ledcAttachPin(ENA2_PIN, motor3Channel);
  ledcAttachPin(ENB2_PIN, motor4Channel);

  // Initialize PWM channels to 0 (motors off)
  ledcWrite(motor1Channel, 0);
  ledcWrite(motor2Channel, 0);
  ledcWrite(motor3Channel, 0);
  ledcWrite(motor4Channel, 0);

  // Initialize PS3 Controller
  Ps3.attach(notify);               // Attach callback function
  Ps3.attachOnConnect(onConnect);   // Attach onConnect function
  Ps3.begin("00:00:00:00:00:00");   // Replace with your PS3 controller's MAC address

  Serial.println("System Initialized. Waiting for PS3 Controller...");
}

// ========================
// Loop Function
// ========================
void loop() {
  // Keep the PS3 controller connection active
  Ps3.loop();
}
