// --- Pin Definitions --- 
#define TRIG_FRONT 6  // Front Ultrasonic Sensor Trigger Pin
#define ECHO_FRONT 7  // Front Ultrasonic Sensor Echo Pin
#define TRIG_RIGHT 8  // Right Ultrasonic Sensor Trigger Pin
#define ECHO_RIGHT 9  // Right Ultrasonic Sensor Echo Pin
#define TRIG_LEFT 10  // Left Ultrasonic Sensor Trigger Pin
#define ECHO_LEFT 11  // Left Ultrasonic Sensor Echo Pin

// Motor Pins
#define MOTOR_LEFT_FORWARD 3
#define MOTOR_LEFT_BACKWARD 4
#define MOTOR_RIGHT_FORWARD 5
#define MOTOR_RIGHT_BACKWARD 2

// --- Constants ---
const int safeDistance = 30; // Safe distance from walls in cm
const int setPoint = 15; // Desired distance from the right wall for PID

// --- PID Constants ---
float Kp = 1.2, Ki = 0.0, Kd = 0.5;
float error, previousError = 0, integral = 0, derivative;
float maxSpeed = 255, baseSpeed = 150; // Motor speeds

// --- Dynamic Maze Grid ---
const int gridRows = 10;
const int gridCols = 10;
int mazeGrid[gridRows][gridCols]; // 2D array for mapping the maze
int currentX = 0, currentY = 0; // Current position in the grid

// --- Function Declarations ---
long readUltrasonicDistance(int trigPin, int echoPin);
void updateMazeWithSensors();
void followRightHandRule();
void maintainDistanceFromWall();
void moveForward();
void turnLeft();
void turnRight();
void turnAround();
void stopMotors();
void adjustMotors(float pidSpeed);

// --- Setup Function ---
void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);
  
  // Setup Ultrasonic Pins
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  // Setup Motor Pins
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
  
  // Initialize maze grid (0 = unexplored, 1 = wall, 2 = path)
  for (int i = 0; i < gridRows; i++) {
    for (int j = 0; j < gridCols; j++) {
      mazeGrid[i][j] = 0;
    }
  }
}

// --- Main Loop ---
void loop() {
  // Update the maze mapping based on sensor readings
  updateMazeWithSensors();

  // Use the Right-Hand Rule for basic maze exploration
  followRightHandRule();
}

// --- Function Definitions ---

// Read distance from ultrasonic sensor
long readUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) / 58;  // Convert time to cm
}

// Update the maze grid with sensor readings
void updateMazeWithSensors() {
  long frontDist = readUltrasonicDistance(TRIG_FRONT, ECHO_FRONT);
  long rightDist = readUltrasonicDistance(TRIG_RIGHT, ECHO_RIGHT);
  long leftDist = readUltrasonicDistance(TRIG_LEFT, ECHO_LEFT);

  // Update grid based on sensor readings
  if (frontDist < safeDistance) {
    mazeGrid[currentX][currentY + 1] = 1; // Wall in front
  } else {
    mazeGrid[currentX][currentY + 1] = 0; // Path in front
  }

  if (rightDist < safeDistance) {
    mazeGrid[currentX + 1][currentY] = 1; // Wall to the right
  } else {
    mazeGrid[currentX + 1][currentY] = 0; // Path to the right
  }

  if (leftDist < safeDistance) {
    mazeGrid[currentX - 1][currentY] = 1; // Wall to the left
  } else {
    mazeGrid[currentX - 1][currentY] = 0; // Path to the left
  }
}

// Implement Right-Hand Rule navigation
void followRightHandRule() {
  long frontDist = readUltrasonicDistance(TRIG_FRONT, ECHO_FRONT);
  long rightDist = readUltrasonicDistance(TRIG_RIGHT, ECHO_RIGHT);
  long leftDist = readUltrasonicDistance(TRIG_LEFT, ECHO_LEFT);

  // Maintain PID-controlled distance from the right wall
  maintainDistanceFromWall();

  // Right-Hand Rule Logic
  if (rightDist > safeDistance) {
    turnRight();
    moveForward();
  } else if (frontDist > safeDistance) {
    moveForward();
  } else if (leftDist > safeDistance) {
    turnLeft();
    moveForward();
  } else {
    turnAround();
  }
}

// Use PID to maintain a consistent distance from the right wall
void maintainDistanceFromWall() {
  long rightDist = readUltrasonicDistance(TRIG_RIGHT, ECHO_RIGHT);

  // Calculate PID values
  error = setPoint - rightDist;
  integral += error;
  derivative = error - previousError;
  float motorSpeedAdjustment = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Adjust motor speeds based on PID output
  adjustMotors(motorSpeedAdjustment);

  previousError = error;
}

// Adjust motor speeds based on PID calculations
void adjustMotors(float pidSpeed) {
  float leftMotorSpeed = baseSpeed + pidSpeed;
  float rightMotorSpeed = baseSpeed - pidSpeed;

  leftMotorSpeed = constrain(leftMotorSpeed, 0, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);

  analogWrite(MOTOR_LEFT_FORWARD, leftMotorSpeed);
  analogWrite(MOTOR_RIGHT_FORWARD, rightMotorSpeed);
}

// Move forward function
void moveForward() {
  analogWrite(MOTOR_LEFT_FORWARD, baseSpeed);
  analogWrite(MOTOR_RIGHT_FORWARD, baseSpeed);
}

// Turn left (90 degrees)
void turnLeft() {
  analogWrite(MOTOR_LEFT_BACKWARD, baseSpeed);
  analogWrite(MOTOR_RIGHT_FORWARD, baseSpeed);
  delay(500);  // Adjust this value for the 90-degree turn timing
  stopMotors();
}

// Turn right (90 degrees)
void turnRight() {
  analogWrite(MOTOR_LEFT_FORWARD, baseSpeed);
  analogWrite(MOTOR_RIGHT_BACKWARD, baseSpeed);
  delay(500);  // Adjust this value for the 90-degree turn timing
  stopMotors();
}

// Turn around (180 degrees)
void turnAround() {
  analogWrite(MOTOR_LEFT_FORWARD, baseSpeed);
  analogWrite(MOTOR_RIGHT_BACKWARD, baseSpeed);
  delay(1000);  // Adjust for 180-degree turn
  stopMotors();
}

// Stop both motors
void stopMotors() {
  analogWrite(MOTOR_LEFT_FORWARD, 0);
  analogWrite(MOTOR_RIGHT_FORWARD, 0);
  analogWrite(MOTOR_LEFT_BACKWARD, 0);
  analogWrite(MOTOR_RIGHT_BACKWARD, 0);
}
