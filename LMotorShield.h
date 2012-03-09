/* This library for Arduino provides convinient interface for LMotorShield extension board
 * for Arduino. Though you may access all LMotorShield hardware via standard Servo
 * library and control motors directly via L293D using digital pins and PWM.
 *
 * Copyright (C) 2011 Artem Borisovskiy (bytefu@gmail.com), http://robocraft.ru
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef LMOTORSHIELD_H_
#define LMOTORSHIELD_H_

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <stdint.h>
#include <Servo.h>

enum
{
  LMS_MOTORS_QUANTITY = 2,
  LMS_SERVOS_QUANTITY = 4
};

enum
{
  LMS_FIRST_MOTOR = 1 << 0,
  LMS_MOTOR1 = LMS_FIRST_MOTOR,
  LMS_MOTOR2 = LMS_FIRST_MOTOR << 1,
  LMS_MOTORS = LMS_MOTOR1 | LMS_MOTOR2,

  LMS_FIRST_SERVO = LMS_FIRST_MOTOR << LMS_MOTORS_QUANTITY,
  LMS_SERVO1 = LMS_FIRST_SERVO << 0,
  LMS_SERVO2 = LMS_FIRST_SERVO << 1,
  LMS_SERVO3 = LMS_FIRST_SERVO << 2,
  LMS_SERVO4 = LMS_FIRST_SERVO << 3,
  LMS_SERVOS = LMS_SERVO1 | LMS_SERVO2 | LMS_SERVO3 | LMS_SERVO4
};

enum LMS_Direction
{
  LMS_FORWARD = HIGH,
  LMS_BACKWARD = LOW
};

class LMotorShield
{
public:
  LMotorShield();
  ~LMotorShield();

  void remapMotorPins(uint8_t motor_num,
    uint8_t pwm_pin, uint8_t dir_pin, uint8_t brk_pin);
  void remapServoPins(uint8_t servo_num, uint8_t signal_pin);

  void begin(unsigned selected_units);
  void end();

  void motorSpeed(uint8_t motor, uint8_t speed);

  void motorDirection(uint8_t motor, LMS_Direction direction);
  void motorForward(uint8_t motor);
  void motorBackward(uint8_t motor);

  void motorBreak(uint8_t motor, bool on);
  void motorStop(uint8_t motor);
  void motorRun(uint8_t motor);

  void multipleMotorSpeed(unsigned selected_motors, uint8_t speed);

  void multipleMotorDirection(unsigned selected_motors, LMS_Direction direction);
  void multipleMotorForward(unsigned selected_motors);
  void multipleMotorBackward(unsigned selected_motors);

  void multipleMotorBreak(unsigned selected_motors, bool on);
  void multipleMotorStop(unsigned selected_motors);
  void multipleMotorRun(unsigned selected_motors);

  void servoWrite(uint8_t servo, uint8_t angle);
  void multipleServoWrite(unsigned selected_servos, uint8_t angle);

  uint8_t getMotorLastSpeed(uint8_t motor);
  LMS_Direction getMotorDirection(uint8_t motor);
  bool getMotorBreak(uint8_t motor);

  uint8_t getServoLastAngle(uint8_t servo);

private:
  struct Motor
  {
    uint8_t pwm_pin;
    uint8_t dir_pin;
    uint8_t brk_pin;
    uint8_t last_speed;
  };

  typedef Motor MotorArray[LMS_MOTORS_QUANTITY];
  MotorArray motors;

  Servo servos[LMS_SERVOS_QUANTITY];

  typedef uint8_t PinArray[LMS_SERVOS_QUANTITY];
  PinArray servo_pins;
};

#endif
