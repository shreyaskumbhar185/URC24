// Define pin numbers for the IR sensors
#define SENSOR_LEFT 2
#define SENSOR_CENTER 3
#define SENSOR_RIGHT 4

// Motor control pins for L298N motor driver
#define MOTOR_LEFT_FORWARD 5
#define MOTOR_LEFT_BACKWARD 6
#define MOTOR_RIGHT_FORWARD 9
#define MOTOR_RIGHT_BACKWARD 10

// Variables for PID control
float Kp = 1.5;  // Proportional gain (adjust to tune)
float Ki = 0.0;  // Integral gain (start with 0)
float Kd = 1.0;  // Derivative gain (adjust to tune)
float error = 0, previousError = 0;
float integral = 0;
float derivative = 0;
float correction = 0;
int baseSpeed = 150;  // Base motor speed (adjust depending on motor power)

// Motor speed limits (adjust depending on desired speed)
int maxSpeed = 255;
int minSpeed = 0;

void setup() {
  // Set up sensor pins as input
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_CENTER, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);

  // Set up motor control pins as output
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);

  Serial.begin(9600);  // Initialize serial communication for debugging
}

void loop() {
  // Read sensor values (LOW when detecting the black line)
  int leftValue = digitalRead(SENSOR_LEFT);
  int centerValue = digitalRead(SENSOR_CENTER);
  int rightValue = digitalRead(SENSOR_RIGHT);

  // Calculate the error based on sensor readings
  if (leftValue == LOW && rightValue == LOW && centerValue == HIGH) {
    error = 0;  // Robot is on track (center sensor detecting the line)
  } 
  else if (leftValue == HIGH && centerValue == LOW && rightValue == LOW) {
    error = -1;  // Robot is veering to the left (left sensor detecting the line)
  } 
  else if (leftValue == LOW && centerValue == LOW && rightValue == HIGH) {
    error = 1;  // Robot is veering to the right (right sensor detecting the line)
  }

  // Calculate PID terms
  integral += error;                           // Accumulate the integral term
  derivative = error - previousError;          // Calculate the derivative term
  correction = (Kp * error) + (Ki * integral) + (Kd * derivative);  // PID formula

  // Update motor speeds based on PID correction
  int leftMotorSpeed = baseSpeed + correction;   // Adjust left motor speed
  int rightMotorSpeed = baseSpeed - correction;  // Adjust right motor speed

  // Constrain motor speeds to within valid limits
  leftMotorSpeed = constrain(leftMotorSpeed, minSpeed, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, minSpeed, maxSpeed);

  // Set motor speeds
  setMotorSpeed(MOTOR_LEFT_FORWARD, MOTOR_LEFT_BACKWARD, leftMotorSpeed);
  setMotorSpeed(MOTOR_RIGHT_FORWARD, MOTOR_RIGHT_BACKWARD, rightMotorSpeed);

  // Store the current error for the next loop iteration (used in derivative)
  previousError = error;

  // Small delay for stability (adjust as needed)
  delay(10);
}

// Function to control motor speed and direction using L298N motor driver
void setMotorSpeed(int forwardPin, int backwardPin, int speed) {
  if (speed >= 0) {
    analogWrite(forwardPin, speed);
    analogWrite(backwardPin, 0);
  } else {
    analogWrite(forwardPin, 0);
    analogWrite(backwardPin, -speed);
  }
}
