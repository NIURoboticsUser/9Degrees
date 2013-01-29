#include <Arduino.h>

#ifndef _MotorController_h
#define _MotorController_h

#define MOTOR_FRONT 0x1
#define MOTOR_LEFT 0x2
#define MOTOR_RIGHT 0x4
#define MOTOR_BACK 0x8
#define MOTOR_ALL (MOTOR_FRONT | MOTOR_BACK | MOTOR_LEFT | MOTOR_RIGHT)

#ifndef MOTOR_ARM_VALUE
  #define MOTOR_ARM_VALUE 100 // Value to arm motors
#endif

#ifndef MOTOR_MIN_SPEED_VALUE
  #define MOTOR_MIN_SPEED_VALUE (MOTOR_ARM_VALUE + 15) // Value for the lowest speed on the motors
#endif

#ifndef MOTOR_MAX_SPEED_VALUE
  #define MOTOR_MAX_SPEED_VALUE 250 // Value for the highest speed on the motors (raw PWM value)
#endif

#ifndef MOTOR_FRONT_PIN
  #define MOTOR_FRONT_PIN 11
#endif

#ifndef MOTOR_RIGHT_PIN
  #define MOTOR_RIGHT_PIN 5
#endif

#ifndef MOTOR_LEFT_PIN
  #define MOTOR_LEFT_PIN 6
#endif

#ifndef MOTOR_BACK_PIN
  #define MOTOR_BACK_PIN 3
#endif

#ifndef MOTOR_FRONT_RAW_OFFSET
  #define MOTOR_FRONT_RAW_OFFSET 0
#endif

#ifndef MOTOR_BACK_RAW_OFFSET
  #define MOTOR_BACK_RAW_OFFSET 0
#endif

#ifndef MOTOR_LEFT_RAW_OFFSET
  #define MOTOR_LEFT_RAW_OFFSET 0
#endif

#ifndef MOTOR_RIGHT_RAW_OFFSET
  #define MOTOR_RIGHT_RAW_OFFSET 0
#endif

// PWM(motor) = (thrust + MOTOR_motor_THRUST_A) / MOTOR_motor_THRUST_B;

#ifndef MOTOR_FRONT_THRUST_A
  #define MOTOR_FRONT_THRUST_A 54
#endif

#ifndef MOTOR_FRONT_THRUST_B
  #define MOTOR_FRONT_THRUST_B 184.93
#endif

#ifndef MOTOR_RIGHT_THRUST_A
  #define MOTOR_RIGHT_THRUST_A 14.33
#endif

#ifndef MOTOR_RIGHT_THRUST_B
  #define MOTOR_RIGHT_THRUST_B 158.48
#endif

#ifndef MOTOR_BACK_THRUST_A
  #define MOTOR_BACK_THRUST_A 16.73
#endif

#ifndef MOTOR_BACK_THRUST_B
  #define MOTOR_BACK_THRUST_B 161.9
#endif

#ifndef MOTOR_LEFT_THRUST_A
  #define MOTOR_LEFT_THRUST_A 34.722
#endif

#ifndef MOTOR_LEFT_THRUST_B
  #define MOTOR_LEFT_THRUST_B 174.17
#endif

#define MOTOR_FRONT_I 0
#define MOTOR_RIGHT_I 1
#define MOTOR_LEFT_I 2
#define MOTOR_BACK_I 3

/**
 * This class allows you to controll the four motors of the quadcopter easily.
 * <c>MotorController</c> is setup for use with HK-50A ESC and Turnigy D3548/6 790KV motors.
 *
 * To refer to the various motors, use the constants <c>MOTOR_FRONT</c>, <c>MOTOR_BACK</c>, <c>MOTOR_LEFT</c>, and <c>MOTOR_RIGHT</c>
 * for the front, back, left, and right motors, respectively.
 * These constants may be bitwise OR'ed together in all functions ask for which motors to use, except for <c>get</c> functions.
 * For example, <c>armMotor(MOTOR_FRONT | MOTOR_BACK);</c> to arm the front and back motors with one function call.
 * The <c>MOTOR_ALL</c> constant can be used to refer to all motors at the same time.
 *
 * The PWM pin defaults may be changed by defining <c>MOTOR_FRONT_PIN</c>, <c>MOTOR_BACK_PIN</c>, <c>MOTOR_LEFT_PIN</c>,
 * and <c>MOTOR_RIGHT_PIN</c> for the front, back, left, and right motors, respectively.
 * Note that these constants must be defined <i>before</i> including <c>MotorController.h</c>
 * 
 * Motors must be armed before they are used, either by arming individual motors via <c>armMotor(MOTOR_?);</c>,
 * or with <c>armMotors();</c> (which is equivalent to <c>armMotor(MOTOR_ALL);</c>).
 * 
 * The ESC may be programmed through this class. Programming occurs on all motors at the same time.
 * To enable programming, define the <c>MOTOR_PROGRAMMING_ENABLED</c> constant before including this file.
 * Programming mode is entered the first time <c>changeSetting()</c> is called, and left when <c>exitProgramming()</c> is called.
 * When modifying multiple settings, you must call the next <c>changeSetting()</c> call immediately after the previous one completes.
 * Automatic programming depends on accurate timing, and any delay between calls may result in inproper programming.
 * Note that both methods block for several seconds, depending on what option and setting is being changed.
 */
class MotorController {
  public:
    /**
     * Instantiates the <c>MotorController</c>.
     */
    MotorController();
    /**
     * Sets the speed of the motor with a value from 0 to 255. Multiple motors may be selected.
     */
    void setMotorSpeed(byte motors, byte speed);
    
    /**
     * Gets the speed of a single motor, returning the same speed passed to <c>setMotorSpeed</c>.
     */
    byte getMotorSpeed(byte motor);
    
    /**
     * Adds a speed to the selected motors.
     * <c>speed</c> may be negative, but the resultant speed
     * of the motors will be clamped between 0 and 255, inclusive.
     */
    void addMotorSpeed(byte motors, short speed);
    
    /**
     * Adds a speed frpm the selected motors.
     * <c>speed</c> may be negative, but the resultant speed
     * of the motors will be clamped between 0 and 255, inclusive.
     */
    void subtractMotorSpeed(byte motors, short speed) { addMotorSpeed(motors, -speed); }
    
    /**
     * Returns true if all of the selected motors are armed, false otherwise.
     */
    boolean isArmed(byte motors);
    
    /**
     * Returns true if all motors are armed, false otherwise.
     * Same as calling <c>isArmed(MOTOR_ALL);</c>.
     */
    boolean isArmed() { return isArmed(MOTOR_ALL); }
    
    /**
     * Arms the selected motor.
     */
    void armMotor(byte motors);
    
    /**
     * Disarms the selected motor.
     * Note that if the motors are currently running, 
     * they will take a few seconds before spinning down.
     */
    void disarmMotor(byte motors);
    
    /**
     * Arms all the motors. Same as calling <c>armMotor(MOTOR_ALL)</c>.
     */
    void armMotors() { armMotor(MOTOR_ALL); }
    
    /**
     * Disarms all the motors. Same as calling <c>disarmMotor(MOTOR_ALL)</c>.
     */
    void disarmMotors() { disarmMotor(MOTOR_ALL); }
    
    /**
     * Gets the raw PWM value that is being sent to the selected motors.
     */
    void setMotorRaw(byte motors, byte raw);
    
    /**
     * Gets the raw PWM value that is being sent to the motor.
     */
    byte getMotorRaw(byte motor);
    
    /**
     * Sets the thrust of the selected motors.
     */
     void setMotorThrust(byte motors, uint16_t thrust);
     
     /**
      * Gets the thrust of the selected motors.
      */
     uint16_t getMotorThrust(byte motor);
    
  private:
    byte armedMask;
    byte motorSpeeds[4];
    byte motorRaw[4];

#ifdef MOTOR_PROGRAMMING_ENABLED
  public:
    
    /**
     * Changes the given option to the given setting. This method blocks for many seconds.
     */
    void changeSetting(byte option, byte setting);
    
    /**
     * Exits programming mode. This method blocks for 5 seconds.
     */
    void exitProgramming();
    
  private:
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
  byte raw = (speed > 0) ? map(speed, 0, 255, MOTOR_MIN_SPEED_VALUE, MOTOR_MAX_SPEED_VALUE) : MOTOR_ARM_VALUE;
  if (motors & MOTOR_FRONT && armedMask & MOTOR_FRONT) {
    motorSpeeds[MOTOR_FRONT_I] = speed;
    setMotorRaw(MOTOR_FRONT, raw + constrain(raw > MOTOR_ARM_VALUE ? MOTOR_FRONT_RAW_OFFSET : 0, 0, 255));
  }
  
  if (motors & MOTOR_BACK && armedMask & MOTOR_BACK) {
    motorSpeeds[MOTOR_BACK_I] = speed;
    setMotorRaw(MOTOR_BACK, raw + constrain(raw > MOTOR_ARM_VALUE ? MOTOR_BACK_RAW_OFFSET : 0, 0, 255));
  }
  
  if (motors & MOTOR_LEFT && armedMask & MOTOR_LEFT) {
    motorSpeeds[MOTOR_LEFT_I] = speed;
    setMotorRaw(MOTOR_LEFT, raw + constrain(raw > MOTOR_ARM_VALUE ? MOTOR_LEFT_RAW_OFFSET : 0, 0, 255));
  }
  
  if (motors & MOTOR_RIGHT && armedMask & MOTOR_RIGHT) {
    motorSpeeds[MOTOR_RIGHT_I] = speed;
    setMotorRaw(MOTOR_RIGHT, raw + constrain(raw > MOTOR_ARM_VALUE ? MOTOR_RIGHT_RAW_OFFSET : 0, 0, 255));
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
  if (motor & MOTOR_FRONT) {
    motorRaw[MOTOR_FRONT_I] = raw;
    if (raw > 0)
      analogWrite(MOTOR_FRONT_PIN, raw);
    else
      digitalWrite(MOTOR_FRONT_PIN, LOW);
  }
  
  if (motor & MOTOR_BACK) {
    motorRaw[MOTOR_BACK_I] = raw;
    if (raw > 0)
      analogWrite(MOTOR_BACK_PIN, raw);
    else
      digitalWrite(MOTOR_BACK_PIN, LOW);
  }
  
  if (motor & MOTOR_LEFT) {
    motorRaw[MOTOR_LEFT_I] = raw;
    if (raw > 0)
      analogWrite(MOTOR_LEFT_PIN, raw);
    else
      digitalWrite(MOTOR_LEFT_PIN, LOW);
  }
  
  if (motor & MOTOR_RIGHT) {
    motorRaw[MOTOR_RIGHT_I] = raw;
    if (raw > 0)
      analogWrite(MOTOR_RIGHT_PIN, raw);
    else
      digitalWrite(MOTOR_RIGHT_PIN, LOW);
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

boolean MotorController::isArmed(byte motors) {
  if (motors & MOTOR_FRONT) {
    if (!(armedMask & MOTOR_FRONT == MOTOR_FRONT)) {
      return false;
    }
  } else if (motors & MOTOR_BACK) {
    if (!(armedMask & MOTOR_BACK == MOTOR_BACK)) {
      return false;
    }
  } else if (motors & MOTOR_LEFT) {
    if (!(armedMask & MOTOR_LEFT == MOTOR_LEFT)) {
      return false;
    }
  } else if (motors & MOTOR_RIGHT) {
    if (!(armedMask & MOTOR_RIGHT == MOTOR_RIGHT)) {
      return false;
    }
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

void MotorController::addMotorSpeed(byte motors, short speed) {

#define _CALC_NEW_SPEED() if (speed > 0) { \
                            if ((short)currentSpeed + speed >= 255) { newSpeed = 255; } \
                            else { newSpeed = currentSpeed + speed; } \
                          } else { \
                            if ((-speed) >= currentSpeed) { newSpeed = 0; } \
                            else { newSpeed = currentSpeed + speed; } \
                          }

  if (speed == 0) {
    return;
  }
  
  byte currentSpeed, newSpeed;
  if (motors & MOTOR_FRONT) {
    currentSpeed = motorSpeeds[MOTOR_FRONT_I];
    
    _CALC_NEW_SPEED()
    
    setMotorSpeed(MOTOR_FRONT, newSpeed);
  }
  
  if (motors & MOTOR_BACK) {
    currentSpeed = motorSpeeds[MOTOR_BACK_I];
    
    _CALC_NEW_SPEED()
    
    setMotorSpeed(MOTOR_BACK, newSpeed);
  }
  
  if (motors & MOTOR_LEFT) {
    currentSpeed = motorSpeeds[MOTOR_LEFT_I];
    
    _CALC_NEW_SPEED()
    
    setMotorSpeed(MOTOR_LEFT, newSpeed);
  }
  
  if (motors & MOTOR_RIGHT) {
    currentSpeed = motorSpeeds[MOTOR_RIGHT_I];
    
    _CALC_NEW_SPEED()
    
    setMotorSpeed(MOTOR_RIGHT, newSpeed);
  }

#undef _CALC_NEW_SPEED
}

void MotorController::setMotorThrust(byte motors, uint16_t thrust) {
  double pwm;
  if (motors & MOTOR_FRONT) {
    pwm = (thrust + MOTOR_FRONT_THRUST_A) / MOTOR_FRONT_THRUST_B;
    setMotorRaw(MOTOR_FRONT, pwm);
    motorSpeeds[MOTOR_FRONT_I] = (pwm >= MOTOR_MIN_SPEED_VALUE ? map(pwm, MOTOR_MIN_SPEED_VALUE, MOTOR_MAX_SPEED_VALUE, 0, 255) : 0);
  }
  
  if (motors & MOTOR_BACK) {
    pwm = (thrust + MOTOR_BACK_THRUST_A) / MOTOR_BACK_THRUST_B;
    setMotorRaw(MOTOR_BACK, pwm);
    motorSpeeds[MOTOR_BACK_I] = (pwm >= MOTOR_MIN_SPEED_VALUE ? map(pwm, MOTOR_MIN_SPEED_VALUE, MOTOR_MAX_SPEED_VALUE, 0, 255) : 0);
  }
  
  if (motors & MOTOR_LEFT) {
    pwm = (thrust + MOTOR_LEFT_THRUST_A) / MOTOR_LEFT_THRUST_B;
    setMotorRaw(MOTOR_LEFT, pwm);
    motorSpeeds[MOTOR_LEFT_I] = (pwm >= MOTOR_MIN_SPEED_VALUE ? map(pwm, MOTOR_MIN_SPEED_VALUE, MOTOR_MAX_SPEED_VALUE, 0, 255) : 0);
  }
  
  if (motors & MOTOR_RIGHT) {
    pwm = (thrust + MOTOR_RIGHT_THRUST_A) / MOTOR_RIGHT_THRUST_B;
    setMotorRaw(MOTOR_RIGHT, pwm);
    motorSpeeds[MOTOR_RIGHT_I] = (pwm >= MOTOR_MIN_SPEED_VALUE ? map(pwm, MOTOR_MIN_SPEED_VALUE, MOTOR_MAX_SPEED_VALUE, 0, 255) : 0);
  }
}

uint16_t MotorController::getMotorThrust(byte motor) {
  double thrust = 0;
  if (motor == MOTOR_FRONT) {
    thrust = motorRaw[MOTOR_FRONT_I] * MOTOR_FRONT_THRUST_B - MOTOR_FRONT_THRUST_A;
  } else if (motor == MOTOR_BACK) {
    thrust = motorRaw[MOTOR_BACK_I] * MOTOR_BACK_THRUST_B - MOTOR_BACK_THRUST_A;
  } else if (motor == MOTOR_LEFT) {
    thrust = motorRaw[MOTOR_LEFT_I] * MOTOR_LEFT_THRUST_B - MOTOR_LEFT_THRUST_A;
  } else if (motor == MOTOR_RIGHT) {
    thrust = motorRaw[MOTOR_RIGHT_I] * MOTOR_RIGHT_THRUST_B - MOTOR_RIGHT_THRUST_A;
  }
  
  return (uint16_t)thrust;
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
  if (programmingMode) {
    setMotorRaw(MOTOR_ALL, 0);
    programmingMode = false;
    delay(5000);
  }
}

#endif // MOTOR_PROGRAMMING_ENABLED

#endif // Double include protection
