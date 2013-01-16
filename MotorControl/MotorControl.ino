
#define MOTOR_PROGRAMMING_ENABLED
#include "MotorController.h"
MotorController controller;

#define INPUT_PIN 1
#define KILL_PIN 12

boolean override = false;

void setup() {
  //controller.armMotors();
  pinMode(12, OUTPUT);
  Serial.begin(9600);
  //delay(5000);
  digitalWrite(12, LOW);
  controller.changeSetting(1, 5);
  controller.changeSetting(3, 3);
  controller.changeSetting(4, 3);
  controller.changeSetting(5, 3);
  digitalWrite(12, HIGH);
  controller.exitProgramming();
  controller.armMotors();
}

void loop() {
  /*short input = analogRead(INPUT_PIN);
  boolean kill = !digitalRead(KILL_PIN);
  byte speed = map(input, 0, 1023, 0, 255);
  if (kill) {
    //controller.disarmMotors();
    //while (true) {}
  } else {
    if (!override)
    controller.setMotorSpeed(MOTOR_ALL, input);
  }*/
  if (!override) {
    /*controller.setMotorRaw(MOTOR_FRONT, 252);
    delay(9800);
    controller.setMotorRaw(MOTOR_FRONT, 200);
    delay(19210);
    //delay(3 * (3 * 350 + 1*930 + 100) + 3 * (3 * 350 + 2*930 + 100) + 3 * (3 * 350 + 3*930 + 100) + (3 * 350 + 4*930 + 100));
    controller.setMotorRaw(MOTOR_FRONT, 252);
    digitalWrite(12, HIGH);
    override = true;
    delay(5000);
    controller.setMotorRaw(MOTOR_FRONT, 0);
    delay(1000);
    controller.armMotor(MOTOR_FRONT);*/
  }
  
  if (Serial.available()) {
    int val = Serial.parseInt();
    override = true;
    controller.setMotorRaw(MOTOR_FRONT, val);
    controller.setMotorRaw(MOTOR_RIGHT, val);
    controller.setMotorRaw(MOTOR_LEFT, val);
    controller.setMotorRaw(MOTOR_BACK, val);
  }
  
  Serial.println(controller.getMotorRaw(MOTOR_FRONT));
  
  
}
