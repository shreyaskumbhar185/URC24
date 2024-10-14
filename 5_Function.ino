
#include <Ps3Controller.h>
#include <ESP32Servo.h>

// Pin Definitions
//HCSR04
#define trigPinFront 18
#define echoPinFront 19
#define trigPinRight 17
#define echoPinRight 5
#define trigPinLeft 4
#define echoPinLeft 16
//L298N
#define ENA 23
#define ENB 13
#define IN1 32
#define IN2 33
#define IN3 25
#define IN4 26
//IR
#define lefts 36
#define middles 39
#define rights 35
// Servo Pins
#define SERVO_PIN 13
#define SERVO_PIN2 14

// PID Constants (if relevant for your control system)
float Kp = 1.5, Ki = 0.1, Kd = 0.3;
float setPoint = 20.0;
float error, lastError, integral, derivative;
float controlSignal;
unsigned long currentTime, lastTime;

// Create servo objects
Servo srvmtr;
Servo srvmtr2;

// Controller Joystick Values
int leftX;
int leftY;
int rightX;
int rightY;

// Separate servo positions for each motor
int servoPos1 = 90;
int servoPos2 = 90;

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
  //HCSR04
  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  //IR
  pinMode(lefts,INPUT);
  pinMode(middles,INPUT);
  pinMode(rights,INPUT);
  //L298N
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Attach servos to pins
  srvmtr.attach(SERVO_PIN);
  srvmtr2.attach(SERVO_PIN2);

  // Home both servos at 90 degrees
  srvmtr.write(servoPos1);
  srvmtr2.write(servoPos2);

  // Print to Serial Monitor
  Serial.println("Ready.");
}

void loop() {
  // Main loop can handle other tasks if needed
  notify();
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
  int middleval = digitalRead(middles);
  int leftval = digitalRead(lefts);
  int rightval = digitalRead(rights);
  
  Serial.print("Left Sensor: ");
  Serial.print(leftval);
  Serial.print(" | Center Sensor: ");
  Serial.print(middleval);
  Serial.print(" | Right Sensor: ");
  Serial.println(rightval);
  
  if (middleval==1 && leftval==0 && rightval==0){
  //error = 0;
  forward(200);
  }
  else if(leftval==1 && middleval==0 && rightval==0) {
  //error = -2;
  left(200);
  }
  else if(leftval==1 && middleval==1 && rightval==0) {
  //error = -1;
  left(150);
  }
  else if (rightval==1 && leftval==0 && middleval==0) { 
  //error = 21;
  right(200);
  }
  else if (rightval==1 && leftval==1 && middleval==0) { 
  //error = +1;
  right(150);
  }
  // else if(rightval==1 && leftval==1 && middleval==1){
  // right(125);
  // delay(500);
  // forward(100);
  // delay(50);
  // }
  else{     
  //error= previous_error;
  backward(100);
  delay(500);
  stop();
  }
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
  if (!Ps3.isConnected())
    return;
  delay(2000);
  // Get joystick values
  leftX = Ps3.data.analog.stick.lx;
  leftY = Ps3.data.analog.stick.ly;
  rightX = Ps3.data.analog.stick.rx;
  rightY = Ps3.data.analog.stick.ry;

  // Control first servo based on left stick
  if (leftY < -100) {
    servoPos1 = 90;
    srvmtr.write(servoPos1);
    delay(10);
  } else {
    if (leftX < -10 && servoPos1 < 180) {
      servoPos1++;
      srvmtr.write(servoPos1);
      delay(10);
    }
    if (leftX > 10 && servoPos1 > 0) {
      servoPos1--;
      srvmtr.write(servoPos1);
      delay(10);
    }
  }

  // Control second servo based on right stick
  if (rightY < -100) {
    servoPos2 = 90;
    srvmtr2.write(servoPos2);
    delay(10);
  } else {
    if (rightX < -10 && servoPos2 < 180) {
      servoPos2++;
      srvmtr2.write(servoPos2);
      delay(10);
    }
    if (rightX > 10 && servoPos2 > 0) {
      servoPos2--;
      srvmtr2.write(servoPos2);
      delay(10);
    }
  }

  // Print to Serial Monitor for debugging
  Serial.print("1X value = ");
  Serial.print(leftX);
  Serial.print(" - 1Y value = ");
  Serial.print(leftY);
  Serial.print(" - 1Servo Pos = ");
  Serial.println(servoPos1);

  Serial.print("2X value = ");
  Serial.print(rightX);
  Serial.print(" - 2Y value = ");
  Serial.print(rightY);
  Serial.print(" - 2Servo Pos = ");
  Serial.println(servoPos2);
}

// Function 4 (triggered by Triangle)
void function4() {
  Serial.println("Function 4 executed");
  // Your code for function 4
  int middleval = digitalRead(middles);
int leftval = digitalRead(lefts);
int rightval = digitalRead(rights);

Serial.print("Left Sensor: ");
Serial.print(leftval);
Serial.print(" | Center Sensor: ");
Serial.print(middleval);
Serial.print(" | Right Sensor: ");
Serial.println(rightval);

if (middleval==0 && leftval==1 && rightval==1){
  //error = 0;
  forward(200);
}
else if(leftval==0 && middleval==1 && rightval==1) {
  //error = -2;
  left(200);
}
else if(leftval==0 && middleval==0 && rightval==1) {
  //error = -1;
  left(150);
}
else if (rightval==0 && leftval==1 && middleval==1) { 
  //error = 21;
  right(200);
}
else if (rightval==0 && leftval==0 && middleval==1) { 
  //error = +1;
  right(150);
}
else if(rightval==0 && leftval==0 && middleval==0){
  right(125);
  delay(500);
  forward(100);
  delay(50);
}
else{     
  //error= previous_error;
  backward(100);
  
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
