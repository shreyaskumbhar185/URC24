// Pin Definitions
#define trigPinFront 18
#define echoPinFront 19
#define trigPinRight 17
#define echoPinRight 5
#define trigPinLeft 4
#define echoPinLeft 16
#define ENA 23
#define ENB 13
#define IN1 32
#define IN2 33
#define IN3 25
#define IN4 26

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
void stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

}

void forward(int speed) {
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
  analogWrite(ENA, speed);  // Speed for motor 1
  analogWrite(ENB, speed);  // Speed for motor 2

}

void backward(int speed) {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
  analogWrite(ENA, speed);  // Speed for motor 1
  analogWrite(ENB, speed);

}

void left(int speed) {
  digitalWrite(IN1, 1); 
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
  analogWrite(ENA, speed/2 );  // Half speed for motor 1 (left)
  analogWrite(ENB, speed); 

}

void right(int speed) {
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
  analogWrite(ENA, speed);      // Full speed for motor 1 (left)
  analogWrite(ENB, speed/2 );
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
    digitalWrite(IN1, LOW); // Slow down left motor
    digitalWrite(IN3, HIGH); // Keep right motor running
  } else {
    // Turn slightly right
    digitalWrite(IN1, HIGH); // Keep left motor running
    digitalWrite(IN3, LOW); // Slow down right motor
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
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
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
  if (distanceFront < 10) {
    stop();
    delay(500);
    left(150);
    delay(500);
  } 
  // If there's no right wall (open space), turn right
  else if (distanceRight > 12.5) {
    stop();
    delay(500);
    right(150);
    delay(500);
  } 
  // If there's a wall to the right, use PID control to follow it
  else {
    PIDControl(distanceRight);
    forward(200);
  }

  delay(100); // Small delay to avoid sensor noise
}
