
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

