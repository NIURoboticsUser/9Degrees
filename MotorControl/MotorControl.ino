
#define MOTOR_PROGRAMMING_ENABLED
#include "MotorController.h"
MotorController controller;

#define INPUT_PIN 1
#define KILL_PIN 12

boolean override = false;
boolean kill = false;

void setup() {
  Serial.begin(9600);
  pinMode(KILL_PIN, INPUT);

#ifdef MOTOR_PROGRAMMING_ENABLED
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  controller.changeSetting(1, 5);
  controller.changeSetting(3, 3);
  controller.changeSetting(4, 3);
  controller.changeSetting(5, 3);
  digitalWrite(13, HIGH);
  controller.exitProgramming();
#else
  delay(5000);
#endif

  controller.armMotors();
  delay(1000);
}

void loop() {
  short input = analogRead(INPUT_PIN);
  if (!kill && digitalRead(KILL_PIN) == LOW) {
    kill = true;
  }
  byte speed = map(input, 0, 1023, 0, 255);
  if (kill) {
    controller.disarmMotors();
  } else {
    short input = analogRead(INPUT_PIN);
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
