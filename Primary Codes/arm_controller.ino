#include <Ps3Controller.h>
#include <ESP32Servo.h>

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

// Servo Pins
#define SERVO_PIN 13
#define SERVO_PIN2 14

// Callback Function
void notify() {
  
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

// On Connection function
void onConnect() {
  // Print to Serial Monitor
  Serial.println("Connected.");
}

void setup() {

  // Setup Serial Monitor for testing
  Serial.begin(115200);

  // Define Callback Function
  Ps3.attach(notify);
  // Define On Connection Function
  Ps3.attachOnConnect(onConnect);
  // Emulate console as specific MAC address (change as required)
  Ps3.begin("00:00:00:00:00:0a");

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
  if (!Ps3.isConnected())
    return;
  delay(2000);
}
