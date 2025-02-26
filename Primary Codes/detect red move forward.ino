/*
#include <Wire.h>
#include "Adafruit_TCS34725.h"

// Create an instance of the TCS34725 sensor with integration time and gain
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  // Start serial communication
  Serial.begin(9600);

  // Check if the TCS34725 sensor is detected
  if (tcs.begin()) {
    Serial.println("TCS34725 found!");
  } else {
    Serial.println("No TCS34725 found... Check your wiring!");
    while (1); // Halt the program if no sensor is found
  }
}

void loop() {
  uint16_t clear, red, green, blue;

  // Get raw data (red, green, blue, clear) from the sensor
  tcs.getRawData(&red, &green, &blue, &clear);

  // Calculate color temperature (in Kelvin) and lux
  uint16_t colorTemp = tcs.calculateColorTemperature(red, green, blue);
  uint16_t lux = tcs.calculateLux(red, green, blue);

  // Print raw RGB and clear values
  Serial.print("Red: "); Serial.print(red); Serial.print("\t");
  Serial.print("Green: "); Serial.print(green); Serial.print("\t");
  Serial.print("Blue: "); Serial.print(blue); Serial.print("\t");
  Serial.print("Clear: "); Serial.print(clear); Serial.print("\t");

  // Print calculated color temperature and lux
  Serial.print("Color Temp: "); Serial.print(colorTemp); Serial.print(" K\t");
  Serial.print("Lux: "); Serial.println(lux);

  delay(1000); // Delay 1 second between readings
}
*/

#include <Wire.h>
#include "Adafruit_TCS34725.h"

// Pins for Motor Driver L298D
const int motor1Pin1 = 7;
const int motor1Pin2 = 6;
const int motor2Pin1 = 5;
const int motor2Pin2 = 4;
// const int motor3Pin1 = 7;
// const int motor3Pin2 = 8;
// const int motor4Pin1 = 9;
// const int motor4Pin2 = 10;

// Set up color sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  Serial.begin(9600);

  // Motor setup
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  // pinMode(motor3Pin1, OUTPUT);
  // pinMode(motor3Pin2, OUTPUT);
  // pinMode(motor4Pin1, OUTPUT);
  // pinMode(motor4Pin2, OUTPUT);

  if (tcs.begin()) {
    Serial.println("Found color sensor");
  } else {
    Serial.println("No TCS34725 found");
    while (1);  // Halt if color sensor is not found
  }
}

void loop() {
  // Read the color values
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  // Normalize values and detect basic colors
  int red = r / 256;
  int green = g / 256;
  int blue = b / 256;

  Serial.print("R: "); Serial.print(red); 
  Serial.print(" G: "); Serial.print(green); 
  Serial.print(" B: "); Serial.println(blue);
  
  // Control motors based on color
  if (red > green && red > blue) {
    // If the color is predominantly red, move forward
    moveForward();
    Serial.println("Moving forward (Red detected)");
  } //else if (green > red && green > blue) {
  //   // If the color is predominantly green, move backward
  //   moveBackward();
  //   Serial.println("Moving backward (Green detected)");
  // } else if (blue > red && blue > green) {
  //   // If the color is predominantly blue, turn left
  //   turnLeft();
  //   Serial.println("Turning left (Blue detected)");
   else {
    // If no color is detected, stop
    stopMotors();
    Serial.println("Motors stopped");
  }

  // delay(500);  // Wait before next color detection
}

// Function to move all motors forward
void moveForward() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  // digitalWrite(motor3Pin1, HIGH);
  // digitalWrite(motor3Pin2, LOW);
  // digitalWrite(motor4Pin1, HIGH);
  // digitalWrite(motor4Pin2, LOW);
}

// Function to move all motors backward
// void moveBackward() {
//   digitalWrite(motor1Pin1, LOW);
//   digitalWrite(motor1Pin2, HIGH);
//   digitalWrite(motor2Pin1, LOW);
//   digitalWrite(motor2Pin2, HIGH);
//   // digitalWrite(motor3Pin1, LOW);
//   // digitalWrite(motor3Pin2, HIGH);
//   // digitalWrite(motor4Pin1, LOW);
//   // digitalWrite(motor4Pin2, HIGH);
// }

// // Function to turn left
// void turnLeft() {
//   digitalWrite(motor1Pin1, LOW);
//   digitalWrite(motor1Pin2, HIGH);
//   digitalWrite(motor2Pin1, LOW);
//   digitalWrite(motor2Pin2, HIGH);
//   // digitalWrite(motor3Pin1, HIGH);
//   // digitalWrite(motor3Pin2, LOW);
//   // digitalWrite(motor4Pin1, HIGH);
//   // digitalWrite(motor4Pin2, LOW);
// }

// Function to stop all motors
void stopMotors() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  // digitalWrite(motor3Pin1, LOW);
  // digitalWrite(motor3Pin2, LOW);
  // digitalWrite(motor4Pin1, LOW);
  // digitalWrite(motor4Pin2, LOW);
}
