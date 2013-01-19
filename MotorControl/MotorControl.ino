
// Uncomment next line if programming
//#define MOTOR_PROGRAMMING_ENABLED

// If on the MEGA, uncomment the next 4 lines
#define MOTOR_FRONT_PIN 2
#define MOTOR_BACK_PIN 4
#define MOTOR_LEFT_PIN 6
#define MOTOR_RIGHT_PIN 8



#include "MotorController.h"
MotorController controller;

#define INPUT_PIN 1

boolean override = false;
boolean kill = false;
short input;

void setup() {
  Serial.begin(9600);

#ifdef MOTOR_PROGRAMMING_ENABLED
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  controller.changeSetting(1, 5);
  controller.changeSetting(3, 3);
  //controller.changeSetting(4, 1); // Clockwise rotation
  controller.changeSetting(4, 2); // Counter-Clockwise rotation
  controller.changeSetting(4, 3);
  controller.changeSetting(5, 3);
  digitalWrite(13, HIGH);
  controller.exitProgramming();
#else
  delay(5000);
  controller.armMotors();
#endif

  
  delay(1000);
}

void loop() {
  if (kill) {
    controller.disarmMotors();
  } else {
    input = analogRead(INPUT_PIN);
    byte speed = map(input, 0, 1023, 0, 255);
    if (!override)
      controller.setMotorSpeed(MOTOR_ALL, input);
  }
  
  if (Serial.available()) {
    int val = Serial.parseInt();
    if (val >= 0 && val <= 255) {
      override = true;
      controller.setMotorRaw(MOTOR_ALL, val);
    } else {
      override = false; 
    }
  }
  
  Serial.println(controller.getMotorRaw(MOTOR_FRONT));
  
  
}
