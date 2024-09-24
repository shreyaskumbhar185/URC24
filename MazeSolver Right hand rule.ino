// Pin Definitions
#define trigPinFront 9
#define echoPinFront 10
#define trigPinRight 11
#define echoPinRight 12
#define trigPinLeft 13
#define echoPinLeft 8
#define motorLeftFwd 2
#define motorLeftBwd 3
#define motorRightFwd 4
#define motorRightBwd 5

// PID Constants
float Kp = 1.5, Ki = 0.1, Kd = 0.3;
float setPoint = 20.0; // Desired distance from the right wall in cm
float error, lastError, integral, derivative;
float controlSignal;

// Time management for PID
unsigned long currentTime, lastTime;

// Function to measure distance using ultrasonic sensor
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  float duration = pulseIn(echoPin, HIGH);
  return (duration * 0.034) / 2; // Convert to cm
}

// Motor control functions
void moveForward() {
  digitalWrite(motorLeftFwd, HIGH);
  digitalWrite(motorLeftBwd, LOW);
  digitalWrite(motorRightFwd, HIGH);
  digitalWrite(motorRightBwd, LOW);
}

void turnLeft() {
  digitalWrite(motorLeftFwd, LOW);
  digitalWrite(motorLeftBwd, HIGH);
  digitalWrite(motorRightFwd, HIGH);
  digitalWrite(motorRightBwd, LOW);
}

void turnRight() {
  digitalWrite(motorLeftFwd, HIGH);
  digitalWrite(motorLeftBwd, LOW);
  digitalWrite(motorRightFwd, LOW);
  digitalWrite(motorRightBwd, HIGH);
}

void stopMotors() {
  digitalWrite(motorLeftFwd, LOW);
  digitalWrite(motorLeftBwd, LOW);
  digitalWrite(motorRightFwd, LOW);
  digitalWrite(motorRightBwd, LOW);
}

// PID controller for right sensor
void PIDControl(float distanceRight) {
  currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;

  // Calculate error
  error = setPoint - distanceRight;
  integral += error * deltaTime;
  derivative = (error - lastError) / deltaTime;

  // Calculate control signal
  controlSignal = Kp * error + Ki * integral + Kd * derivative;

  // Adjust motor speeds based on control signal
  if (controlSignal > 0) {
    // Turn slightly left
    digitalWrite(motorLeftFwd, LOW); // Slow down left motor
    digitalWrite(motorRightFwd, HIGH); // Keep right motor running
  } else {
    // Turn slightly right
    digitalWrite(motorLeftFwd, HIGH); // Keep left motor running
    digitalWrite(motorRightFwd, LOW); // Slow down right motor
  }

  // Update variables for next iteration
  lastError = error;
  lastTime = currentTime;
}

void setup() {
  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);

  pinMode(motorLeftFwd, OUTPUT);
  pinMode(motorLeftBwd, OUTPUT);
  pinMode(motorRightFwd, OUTPUT);
  pinMode(motorRightBwd, OUTPUT);
  
  Serial.begin(9600);
  lastTime = millis();
}

void loop() {
  float distanceFront = getDistance(trigPinFront, echoPinFront);
  float distanceRight = getDistance(trigPinRight, echoPinRight);
  float distanceLeft = getDistance(trigPinLeft, echoPinLeft);

  Serial.print("Front: ");
  Serial.print(distanceFront);
  Serial.print(" Right: ");
  Serial.print(distanceRight);
  Serial.print(" Left: ");
  Serial.println(distanceLeft);

  // If front is blocked, turn left
  if (distanceFront < 15) {
    stopMotors();
    delay(500);
    turnLeft();
    delay(500);
  } 
  // If there's no right wall (open space), turn right
  else if (distanceRight > 30) {
    stopMotors();
    delay(500);
    turnRight();
    delay(500);
  } 
  // If there's a wall to the right, use PID control to follow it
  else {
    PIDControl(distanceRight);
    moveForward();
  }

  delay(100); // Small delay to avoid sensor noise
}
