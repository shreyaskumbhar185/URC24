#include <Ps3Controller.h>

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

// PID Constants (if relevant for your control system)
float Kp = 1.5, Ki = 0.1, Kd = 0.3;
float setPoint = 20.0;
float error, lastError, integral, derivative;
float controlSignal;
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

// Function declarations
void function1();
void function2();
void function3();
void function4();
void function5();

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
  // Initialize serial for debugging
  Serial.begin(115200);
  lastTime = millis();
  
  // Initialize PS3 Controller
  Ps3.attach(notify);             // Attach callback function for button events
  Ps3.attachOnConnect(onConnect); // Attach onConnect function (optional)
  Ps3.begin("00:00:00:00:00:00"); // Replace with your PS3 controller's MAC address

  Serial.println("System Initialized. Waiting for PS3 Controller...");

  // Initialize pins for functions
  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void loop() {
  // Main loop can handle other tasks if needed
}

// Callback function to handle button events
void notify() {
  // Check if X is pressed
  if (Ps3.event.button_down.cross) {
    function1();
  }
  // Check if O (Circle) is pressed
  else if (Ps3.event.button_down.circle) {
    function2();
  }
  // Check if Square is pressed
  else if (Ps3.event.button_down.square) {
    function3();
  }
  // Check if Triangle is pressed
  else if (Ps3.event.button_down.triangle) {
    function4();
  }
  // Check if R1 is pressed
  else if (Ps3.event.button_down.r1) {
    function5();
  }
}

// Function 1 (triggered by X)
void function1() {
  Serial.println("Function 1 executed");
  // Your code for function 1
}

// Function 2 (triggered by O/Circle)
void function2() {

  Serial.println("Function 2 executed");
  // Your code for function 2
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

// Function 3 (triggered by Square)
void function3() {
  Serial.println("Function 3 executed");
  // Your code for function 3
}

// Function 4 (triggered by Triangle)
void function4() {
  Serial.println("Function 4 executed");
  // Your code for function 4
}

// Function 5 (triggered by R1)
void function5() {
  Serial.println("Function 5 executed");
  // Your code for function 5
}

// Optional callback function for when the PS3 controller connects
void onConnect() {
  Serial.println("PS3 Controller connected");
}
//Switch Case made by Yashodhan K 

// Motor control functions
//            motor left                    motor right
//               ENA                            ENB
//               IN1 (A)                        IN3 (A)   
//               IN2 (C)                        IN4 (C)  
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
