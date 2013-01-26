/*
                   y
                   ^
                   |
                   1<--REGULAR (spins CCW)
                   |
                   |
                   |
                   |
     4-------------------------2--->  x
     ^             |                  ^
     |_____PUSHER  |                  |_________PUSHER (spins CW)
                   |
                   |
                   3<--REGULAR
                   
 */
#include <SoftwareSerial.h>
SoftwareSerial dofSerial(11, 12); // RX, TX

#define MOTOR_FRONT_PIN 3
#define MOTOR_RIGHT_PIN 5
#define MOTOR_BACK_PIN 6
//#define MOTOR_LEFT_PIN 8
#define MOTOR_LEFT_PIN 10

//#define MOTOR_LEFT_RAW_OFFSET 15
//#define MOTOR_PROGRAMMING_ENABLED
#include "MotorController.h"
MotorController controller;

#include "DofData.h"
#include "DofHandler.h"
DofHandler<SoftwareSerial> dofHandler(&dofSerial);
//DofHandler<HardwareSerial> dofHandler(&Serial1);
#define INCREMENT 2
#define WTHRESHOLD 0.02
#define CHANGE_MULTIPLIER 25

// Trig and interp defines
#define COPTER_RADIUS 125
#define BALANCE_INTERP_MULTIPIER 0.1
#define BALANCE_INTERP_THRESHOLD 0.1

double wx, wy, wz;
double targetSpeed = 0;
unsigned long lastSpeedChangeTime;

boolean kill, noDof;

//positive wx corresponds to CW rotation around the x-axis (motor 3 goes up, 1 goes down)
//positive wy corresponds to CW rotation around the y-axis (motor 2 goes up, 4 goes down)
//positive wz corresponds to CW rotation around the z-axis (looking from above)

//int motorValues[]={110, 110, 110, 110}; //Motors 1, 2, 3, and 4
int motorPins[]={3,5,6,9}; //PWM pins

unsigned long lastCollectionTime;

void setup(){
  Serial.begin(9600);
  //arm motors
  
  
  #ifdef MOTOR_PROGRAMMING_ENABLED
  //delay(5000);
  Serial.println("Setting 1...");
  //controller.changeSetting(1, 5);
  Serial.println("Setting 2...");
  controller.changeSetting(2, 1);
  //controller.changeSetting(3, 4);
  Serial.println("Setting 3...");
  //controller.changeSetting(4, 1); // Clockwise rotation
  //controller.changeSetting(4, 2); // Counter-Clockwise rotation
  Serial.println("Setting 4...");
  //controller.changeSetting(4, 3);
  Serial.println("Setting 5...");
  //controller.changeSetting(5, 3);
  Serial.println("Done");
  controller.exitProgramming();
  delay(5000);
  controller.armMotors();
  delay(2000);
  controller.disarmMotors();
  delay(500000);
  #endif
  //writeAll(0);
  //delay(1000);
  //start motors up a bit
  //writeAll(127); //ARM
  controller.disarmMotors();
  delay(3000);
  controller.armMotors();
  delay(1000);
  
  dofHandler.begin(9600, 28800);
  dofHandler.setDataMode(DOF_DATA_MODE_EULER);
  dofHandler.zeroCalibrate();
  dofHandler.setUpdateInterval(40);
  dofHandler.requestData();
  Serial.println("Ready");
  lastCollectionTime = millis();
  lastSpeedChangeTime = millis();
}

void loop(){
  
  if(Serial.available()){
    if (Serial.peek() == 'z') {
      Serial.read();
      dofHandler.zeroCalibrate();
    } else if (Serial.peek() == 'k') {
      Serial.read();
      kill = !kill;
      noDof = kill;
      controller.setMotorSpeed(MOTOR_ALL, 0);
    } else if (Serial.peek() == 's') {
      noDof = true;
      Serial.println("Dropping");
      for (int i = 0; i < 20; i++) {
        controller.subtractMotorSpeed(MOTOR_ALL, 13);
        delay(100);
      }
      Serial.println("Dropped");
      noDof = true;
    } else if (Serial.peek() == 'r') {
      dofHandler.zeroCalibrate();
      kill = false;
      noDof = false;
    }
 
    static char charray[10];
    memset(charray, 0, 9);
   
    for(int i=0; i<10 && Serial.available(); i++){
      delay(5);
      charray[i]=Serial.read();
    }
    
    int num = atoi(charray);
    
    //for(int i=MOTOR_FRONT; i<=MOTOR_BACK; i++) motorValues[i]+=num;
    controller.setMotorSpeed(MOTOR_ALL, num);
    //targetSpeed = constrain(targetSpeed + num, 0, 255);
    
  }
  Serial.println(controller.getMotorSpeed(MOTOR_FRONT));
  
  dofHandler.checkStreamValid();
  if (false && !noDof && dofHandler.isNewDataAvailable(true)) {
    unsigned long now = millis();
    //Serial.print("Time: "); Serial.println(now - lastCollectionTime);
    dofHandler.requestData();
    lastCollectionTime = now;
    EulerData eulerData = dofHandler.getEulerData();
    wx = -eulerData.pitch;
    wy = -eulerData.roll;
    wz = eulerData.yaw;
    
    double wx_h, wy_h, wz_h;
    wx_h = sin(eulerData.pitch) * COPTER_RADIUS;
    wy_h = sin(eulerData.roll) * COPTER_RADIUS;
    wz_h = sin(eulerData.yaw) * COPTER_RADIUS;
    
    // Front, right, back, left
    double speeds[4] = {targetSpeed - wx_h, targetSpeed - wy_h, targetSpeed + wx_h, targetSpeed + wy_h };
    
    // Get lowest speed
    double smallestSpeed = speeds[0];
    for (int i = 1; i < 4; i++) {
      if (speeds[i] < smallestSpeed) {
        smallestSpeed = speeds[i];
      }
    }
    
    // Slowly balance the copter, but only if there is something significant to balance (prevent doing extra unneeded work)
    if (abs(targetSpeed - smallestSpeed) >= BALANCE_INTERP_THRESHOLD) {
      // Interpolate to balance
      now = millis();
      double timeSinceLastSpeedChange = (now - lastSpeedChangeTime) / 1000.0;
      lastSpeedChangeTime = now;
      for (int i = 0; i < 4; i++) {
        double spd = speeds[i];
        speeds[i] -= ((spd - smallestSpeed) * BALANCE_INTERP_MULTIPIER * timeSinceLastSpeedChange);
      }
    }
    
    // Yaw
    speeds[0] += wz_h;
    speeds[2] += wz_h;
    speeds[1] -= wz_h;
    speeds[3] -= wz_h;
    
    for (int i = 0; i < 4; i++) {
      speeds[i] = constrain(speeds[i], 0, 255);
    }
    
    controller.setMotorSpeed(MOTOR_FRONT, speeds[0]);
    controller.setMotorSpeed(MOTOR_RIGHT, speeds[1]);
    controller.setMotorSpeed(MOTOR_BACK, speeds[2]);
    controller.setMotorSpeed(MOTOR_LEFT, speeds[3]);
    
    
    //dofHandler.printData(Serial);
    /*double abs_wx = abs(wx) - WTHRESHOLD;
    double abs_wy = abs(wy) - WTHRESHOLD;
    double abs_wz = abs(wz) - WTHRESHOLD;
    if(abs(wx) > WTHRESHOLD) {
  
      if(wx>0){  //motor 1 needs to increase, 3 must decrease
        controller.addMotorSpeed(MOTOR_FRONT, abs_wx*CHANGE_MULTIPLIER);
        controller.subtractMotorSpeed(MOTOR_BACK, abs_wx*CHANGE_MULTIPLIER);
        //motorValues[0] += INCREMENT;
        //motorValues[2] -= INCREMENT;
      }
    
      if(wx<0){ //motor 1 needs to decrease, 3 must increase
        controller.subtractMotorSpeed(MOTOR_FRONT, abs_wx*CHANGE_MULTIPLIER);
        controller.addMotorSpeed(MOTOR_BACK, abs_wx*CHANGE_MULTIPLIER);
        //motorValues[0] -= INCREMENT;
        //motorValues[2] += INCREMENT;
      }
    
    }
    
    if(abs(wy)>WTHRESHOLD){
      //Serial.println("wy");
      if(wy>0){ //motor 2 must decrease, 4 must increase
        controller.subtractMotorSpeed(MOTOR_RIGHT, abs_wy*CHANGE_MULTIPLIER);
        controller.addMotorSpeed(MOTOR_LEFT, abs_wy*CHANGE_MULTIPLIER);
        //motorValues[1] -= INCREMENT;
        //motorValues[3] += INCREMENT;
      }
    
      if(wy<0){  //motor 2 must increase, 4 must decrease
        //motorValues[1] += INCREMENT;
        //motorValues[3] -= INCREMENT;
        controller.addMotorSpeed(MOTOR_RIGHT, abs_wy*CHANGE_MULTIPLIER);
        controller.subtractMotorSpeed(MOTOR_LEFT, abs_wy*CHANGE_MULTIPLIER);
      }
    }
    
    if(abs(wz)>WTHRESHOLD){
      //Serial.println("wz");
      if(wz>0){ //clockwise rotation: motors 2 and 4 decrease, 1 and 3 increase
        controller.subtractMotorSpeed(MOTOR_RIGHT, abs_wz*CHANGE_MULTIPLIER);
        controller.subtractMotorSpeed(MOTOR_LEFT, abs_wz*CHANGE_MULTIPLIER);
        controller.addMotorSpeed(MOTOR_FRONT, abs_wz*CHANGE_MULTIPLIER);
        controller.addMotorSpeed(MOTOR_BACK, abs_wz*CHANGE_MULTIPLIER);
      }
    
      if(wz<0){ //CCW rotation: motors 1 and 3 decrease, 2 and 4 increase
        controller.addMotorSpeed(MOTOR_RIGHT, abs_wz*CHANGE_MULTIPLIER);
        controller.addMotorSpeed(MOTOR_LEFT, abs_wz*CHANGE_MULTIPLIER);
        controller.subtractMotorSpeed(MOTOR_FRONT, abs_wz*CHANGE_MULTIPLIER);
        controller.subtractMotorSpeed(MOTOR_BACK, abs_wz*CHANGE_MULTIPLIER);
      }
    }*/
    /*for(int i=MOTOR_FRONT; true && i<=MOTOR_BACK; i *= 2){
      //analogWrite(motorPins[i], motorValues[i]);
      
      Serial.print("MOTOR ");
      Serial.print(i);
      Serial.print(": ");
      //Serial.print(motorValues[i]);
      Serial.print(controller.getMotorRaw(i));
      Serial.print(",  ");
    }
    Serial.println();*/
    Serial.print("Yaw: "); Serial.print(eulerData.yaw);
    Serial.print(", Pitch: "); Serial.print(eulerData.pitch);
    Serial.print(", Roll: "); Serial.print(eulerData.roll);
    Serial.print(", Front: "); Serial.print(controller.getMotorSpeed(MOTOR_FRONT));
    Serial.print(", Back: "); Serial.print(controller.getMotorSpeed(MOTOR_BACK));
    Serial.print(", Left: "); Serial.print(controller.getMotorSpeed(MOTOR_LEFT));
    Serial.print(", Right: "); Serial.print(controller.getMotorSpeed(MOTOR_RIGHT));
    Serial.println();
    //delay(50);
  }
  if (kill) {
    Serial.println("KILL");
  }
}

int smallestIndex(double ary[], int size) {
  int smallest = 0;
  for (int i = 1; i < size; i++) {
    if (ary[i] < ary[smallest]) {
      smallest = i;
    }
  }
  return smallest;
}

/*void writeAll(int value) {
  for(int i=0; i<4; i++){
    motorValues[i] = value;
    analogWrite(motorPins[i], value);
  }
}*/
