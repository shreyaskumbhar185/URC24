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

// Function declarations
void function1();
void function2();
void function3();
void function4();
void function5();

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);

  // Initialize PS3 Controller
  Ps3.attach(notify);             // Attach callback function for button events
  Ps3.attachOnConnect(onConnect); // Attach onConnect function (optional)
  Ps3.begin("00:00:00:00:00:00"); // Replace with your PS3 controller's MAC address

  Serial.println("System Initialized. Waiting for PS3 Controller...");

  // Initialize pins for functions
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
