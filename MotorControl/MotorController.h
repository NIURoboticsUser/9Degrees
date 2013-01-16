#include <Arduino.h>

#ifndef _MotorController_h
#define _MotorController_h

#ifndef MOTOR_FRONT
  #define MOTOR_FRONT 0x1
#endif

#ifndef MOTOR_LEFT
  #define MOTOR_LEFT 0x2
#endif

#ifndef MOTOR_RIGHT
  #define MOTOR_RIGHT 0x4
#endif

#ifndef MOTOR_BACK
  #define MOTOR_BACK 0x8
#endif

#define MOTOR_ALL (MOTOR_FRONT | MOTOR_BACK | MOTOR_LEFT | MOTOR_RIGHT)

#ifndef MOTOR_ARM_VALUE
  #define MOTOR_ARM_VALUE 127 // Value to arm motors
#endif

#ifndef MOTOR_MIN_SPEED_VALUE
  #define MOTOR_MIN_SPEED_VALUE 150 // Minimum value to be armed
#endif

#ifndef MOTOR_FRONT_PIN
  #define MOTOR_FRONT_PIN 3
#endif

#ifndef MOTOR_RIGHT_PIN
  #define MOTOR_RIGHT_PIN 5
#endif

#ifndef MOTOR_LEFT_PIN
  #define MOTOR_LEFT_PIN 6
#endif

#ifndef MOTOR_BACK_PIN
  #define MOTOR_BACK_PIN 10
#endif

#define MOTOR_FRONT_I 0
#define MOTOR_RIGHT_I 1
#define MOTOR_LEFT_I 2
#define MOTOR_BACK_I 3

class MotorController {
  public:
    MotorController();
    void setMotorSpeed(byte motors, byte speed);
    byte getMotorSpeed(byte motor);
    
    boolean isArmed(byte motors);
    boolean isArmed() { return isArmed(MOTOR_ALL); }
    void armMotor(byte motors);
    void disarmMotor(byte motors);
    void armMotors() { armMotor(MOTOR_ALL); }
    void disarmMotors() { disarmMotor(MOTOR_ALL); }
    
    void setMotorRaw(byte motor, byte raw);
    byte getMotorRaw(byte motor);
    
#ifdef MOTOR_PROGRAMMING_ENABLED
    void changeSetting(byte option, byte setting);
    void exitProgramming();
    
#endif
    
  private:
    byte armedMask;
    byte motorSpeeds[4];
    byte motorRaw[4];

#ifdef MOTOR_PROGRAMMING_ENABLED
    boolean programmingMode;
#endif
};

MotorController::MotorController() {
  pinMode(MOTOR_FRONT_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_PIN, OUTPUT);
  pinMode(MOTOR_BACK_PIN, OUTPUT);
  armedMask = 0;
  motorSpeeds[0] = 0;
  motorSpeeds[1] = 0;
  motorSpeeds[2] = 0;
  motorSpeeds[3] = 0;
  
  motorRaw[0] = 0;
  motorRaw[1] = 0;
  motorRaw[2] = 0;
  motorRaw[3] = 0;
}

void MotorController::setMotorSpeed(byte motors, byte speed) {
  byte raw = (speed > 0) ? map(speed, 0, 255, MOTOR_ARM_VALUE, 252) : MOTOR_ARM_VALUE;
  if (motors & MOTOR_FRONT && armedMask & MOTOR_FRONT) {
    motorSpeeds[MOTOR_FRONT_I] = speed;
    setMotorRaw(MOTOR_FRONT, raw);
  }
  
  if (motors & MOTOR_BACK && armedMask & MOTOR_BACK) {
    motorSpeeds[MOTOR_BACK_I] = speed;
    setMotorRaw(MOTOR_BACK, raw);
  }
  
  if (motors & MOTOR_LEFT && armedMask & MOTOR_LEFT) {
    motorSpeeds[MOTOR_LEFT_I] = speed;
    setMotorRaw(MOTOR_LEFT, raw);
  }
  
  if (motors & MOTOR_RIGHT && armedMask & MOTOR_RIGHT) {
    motorSpeeds[MOTOR_RIGHT_I] = speed;
    setMotorRaw(MOTOR_RIGHT, raw);
  }
}

byte MotorController::getMotorSpeed(byte motor) {
  if (motor == MOTOR_FRONT) {
    return motorSpeeds[MOTOR_FRONT_I];
  } else if (motor == MOTOR_BACK) {
    return motorSpeeds[MOTOR_BACK_I];
  } else if (motor == MOTOR_LEFT) {
    return motorSpeeds[MOTOR_LEFT_I];
  } else if (motor == MOTOR_RIGHT) {
    return motorSpeeds[MOTOR_RIGHT_I];
  }
  return 0;
}

void MotorController::setMotorRaw(byte motor, byte raw) {
  if (motor == MOTOR_ALL) {
    setMotorRaw(MOTOR_FRONT, raw);
    setMotorRaw(MOTOR_BACK, raw);
    setMotorRaw(MOTOR_RIGHT, raw);
    setMotorRaw(MOTOR_LEFT, raw);
    return;
  }
  if (motor == MOTOR_FRONT) {
    motorRaw[MOTOR_FRONT_I] = raw;
    analogWrite(MOTOR_FRONT_PIN, raw);
    
  } else if (motor == MOTOR_BACK) {
    motorRaw[MOTOR_BACK_I] = raw;
    analogWrite(MOTOR_BACK_PIN, raw);
    
  } else if (motor == MOTOR_LEFT) {
    motorRaw[MOTOR_LEFT_I] = raw;
    analogWrite(MOTOR_LEFT_PIN, raw);
    
  } else if (motor == MOTOR_RIGHT) {
    motorRaw[MOTOR_RIGHT_I] = raw;
    analogWrite(MOTOR_RIGHT_PIN, raw);
  }
}

byte MotorController::getMotorRaw(byte motor) {
  if (motor == MOTOR_FRONT) {
    return motorRaw[MOTOR_FRONT_I];
  } else if (motor == MOTOR_BACK) {
    return motorRaw[MOTOR_BACK_I];
  } else if (motor == MOTOR_LEFT) {
    return motorRaw[MOTOR_LEFT_I];
  } else if (motor == MOTOR_RIGHT) {
    return motorRaw[MOTOR_RIGHT_I];
  }
  
  return 0;
}

inline boolean MotorController::isArmed(byte motors) {
  if (motors & MOTOR_FRONT) {
    return !(armedMask & MOTOR_FRONT == MOTOR_FRONT);
  } else if (motors & MOTOR_BACK) {
    return !(armedMask & MOTOR_BACK == MOTOR_BACK);
  } else if (motors & MOTOR_LEFT) {
    return !(armedMask & MOTOR_LEFT == MOTOR_LEFT);
  } else if (motors & MOTOR_RIGHT) {
    return !(armedMask & MOTOR_RIGHT == MOTOR_RIGHT);
  }
}

void MotorController::armMotor(byte motors) {
  if (motors & MOTOR_FRONT) {
    motorSpeeds[MOTOR_FRONT_I] = 0;
    armedMask |= MOTOR_FRONT;
    setMotorRaw(MOTOR_FRONT, MOTOR_ARM_VALUE);
  }
  
  if (motors & MOTOR_BACK) {
    motorSpeeds[MOTOR_BACK_I] = 0;
    armedMask |= MOTOR_BACK;
    setMotorRaw(MOTOR_BACK, MOTOR_ARM_VALUE);
  }
  
  if (motors & MOTOR_LEFT) {
    motorSpeeds[MOTOR_LEFT_I] = 0;
    armedMask |= MOTOR_LEFT;
    setMotorRaw(MOTOR_LEFT, MOTOR_ARM_VALUE);
  }
  
  if (motors & MOTOR_RIGHT) {
    motorSpeeds[MOTOR_RIGHT_I] = 0;
    armedMask |= MOTOR_RIGHT;
    setMotorRaw(MOTOR_RIGHT, MOTOR_ARM_VALUE);
  }
}

void MotorController::disarmMotor(byte motors) {
  if (motors & MOTOR_FRONT) {
    motorSpeeds[MOTOR_FRONT_I] = 0;
    armedMask &= ~(MOTOR_FRONT);
    setMotorRaw(MOTOR_FRONT, 0);
  }
  
  if (motors & MOTOR_BACK) {
    motorSpeeds[MOTOR_BACK_I] = 0;
    armedMask &= ~(MOTOR_BACK);
    setMotorRaw(MOTOR_BACK, 0);
  }
  
  if (motors & MOTOR_LEFT) {
    motorSpeeds[MOTOR_LEFT_I] = 0;
    armedMask &= ~(MOTOR_LEFT);
    setMotorRaw(MOTOR_LEFT, 0);
  }
  
  if (motors & MOTOR_RIGHT) {
    motorSpeeds[MOTOR_RIGHT_I] = 0;
    armedMask &= ~(MOTOR_RIGHT);
    setMotorRaw(MOTOR_RIGHT, 0);
  }
}


#ifdef MOTOR_PROGRAMMING_ENABLED

void MotorController::changeSetting(byte option, byte setting) {
  if (!programmingMode) {
    programmingMode = true;
    setMotorRaw(MOTOR_ALL, 0);
    delay(5000);
    setMotorRaw(MOTOR_ALL, 252);
  }
  
  // Go to the option
  for (int i = 1; i <= option; i++) {
    byte current = (i == option) ? 1 : 3;
    delay(current * (700 + 350 * i + 100));
  }
  
  
  // Select the option
  setMotorRaw(MOTOR_ALL, 200);
  
  // Go to the setting
  for (int i = 1; i <= setting; i++) {
    byte current = (i == setting) ? 1 : 3;
    delay(current * (option * 350 + i*930 + 100));
  }
  
  // Select the setting
  setMotorRaw(MOTOR_ALL, 252);
  
  delay(2450);
  
  
}

void MotorController::exitProgramming() {
  setMotorRaw(MOTOR_ALL, 0);
  programmingMode = false;
  delay(5000);
}

#endif


#endif
