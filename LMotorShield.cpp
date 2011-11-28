/* LMotorShield library implementation.
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

#include <stdint.h>
#include <WProgram.h>
#include "LMotorShield.h"


LMotorShield::LMotorShield()
{
  static MotorArray const motors_initial_config =
  {
    { 11, 8, 4 },
    { 3, 7, 2 }
  };
  
  for (unsigned i = 0; i < LMS_MOTORS_QUANTITY; ++i)
    motors[i] = motors_initial_config[i];
  
  static const PinArray servo_pins_initial_config = { 10, 9, 6, 5 };
  
  for (unsigned i = 0; i < LMS_SERVOS_QUANTITY; ++i)
    servo_pins[i] = servo_pins_initial_config[i];
}


LMotorShield::~LMotorShield()
{
}


void LMotorShield::remapMotorPins(uint8_t motor_num,
  uint8_t pwm_pin, uint8_t dir_pin, uint8_t brk_pin)
{
  if (1 < motor_num && motor_num <= LMS_MOTORS_QUANTITY)
  {
    Motor &m = motors[motor_num - 1];
    
    m.pwm_pin = pwm_pin;
    m.dir_pin = dir_pin;
    m.brk_pin = brk_pin;
    m.last_speed = 0;
  }
}


void LMotorShield::remapServoPins(uint8_t servo_num, uint8_t signal_pin)
{
  if (1 <= servo_num && servo_num <= LMS_SERVOS_QUANTITY)
    servo_pins[servo_num - 1] = signal_pin;
}


void LMotorShield::begin(unsigned selected_units)
{
  for (unsigned i = 0, motor_mask = LMS_FIRST_MOTOR;
       i < LMS_MOTORS_QUANTITY;
       ++i, motor_mask <<= 1)
  {
    if (selected_units & motor_mask)
    {
      pinMode(motors[i].dir_pin, OUTPUT);
      pinMode(motors[i].brk_pin, OUTPUT);
    }
  }
  
  for (unsigned i = 0, servo_mask = LMS_FIRST_SERVO;
       i < LMS_SERVOS_QUANTITY;
       ++i, servo_mask <<= 1)
  {
    if (selected_units & servo_mask)
      servos[i].attach(servo_pins[i]);
  }
}


void LMotorShield::end()
{
}


void LMotorShield::motorSpeed(uint8_t motor, uint8_t speed)
{
  if (1 <= motor && motor <= LMS_MOTORS_QUANTITY)
  {
    motors[motor - 1].last_speed = speed;
    analogWrite(motors[motor - 1].pwm_pin, speed);
  }
}


void LMotorShield::motorDirection(uint8_t motor, LMS_Direction direction)
{
  if (1 <= motor && motor <= LMS_MOTORS_QUANTITY)
    digitalWrite(motors[motor - 1].dir_pin, direction);
}


void LMotorShield::motorForward(uint8_t motor)
{
  motorDirection(motor, LMS_FORWARD);
}


void LMotorShield::motorBackward(uint8_t motor)
{
  motorDirection(motor, LMS_BACKWARD);
}


void LMotorShield::motorBreak(uint8_t motor, bool on)
{
  if (1 <= motor && motor <= LMS_MOTORS_QUANTITY)
    digitalWrite(motors[motor - 1].brk_pin, (on ? HIGH : LOW));
}


void LMotorShield::motorStop(uint8_t motor)
{
  motorBreak(motor, true);
}


void LMotorShield::motorRun(uint8_t motor)
{
  motorBreak(motor, false);
}


void LMotorShield::multipleMotorSpeed(unsigned selected_motors, uint8_t speed)
{
  for (unsigned i = 1, motor_mask = LMS_FIRST_MOTOR;
       i <= LMS_MOTORS_QUANTITY;
       ++i, motor_mask <<= 1)
  {
    if (selected_motors & motor_mask)
      motorSpeed(i, speed);
  }
}


void LMotorShield::multipleMotorDirection(
 unsigned selected_motors, LMS_Direction direction)
{
  for (unsigned i = 1, motor_mask = LMS_FIRST_MOTOR;
       i <= LMS_MOTORS_QUANTITY;
       ++i, motor_mask <<= 1)
  {
    if (selected_motors & motor_mask)
      motorDirection(i, direction);
  }
}


void LMotorShield::multipleMotorForward(unsigned selected_motors)
{
  multipleMotorDirection(selected_motors, LMS_FORWARD);
}


void LMotorShield::multipleMotorBackward(unsigned selected_motors)
{
  multipleMotorDirection(selected_motors, LMS_BACKWARD);
}


void LMotorShield::multipleMotorBreak(unsigned selected_motors, bool on)
{
  for (unsigned i = 1, motor_mask = LMS_FIRST_MOTOR;
       i <= LMS_MOTORS_QUANTITY;
       ++i, motor_mask <<= 1)
  {
    if (selected_motors & motor_mask)
      motorBreak(i, on);
  }
}


void LMotorShield::multipleMotorStop(unsigned selected_motors)
{
  multipleMotorBreak(selected_motors, true);
}


void LMotorShield::multipleMotorRun(unsigned selected_motors)
{
  multipleMotorBreak(selected_motors, false);
}


void LMotorShield::servoWrite(uint8_t servo, uint8_t angle)
{
  if (1 <= servo && servo <= LMS_SERVOS_QUANTITY)
    servos[servo - 1].write(angle);
}


void LMotorShield::multipleServoWrite(unsigned selected_servos, uint8_t angle)
{
  for (unsigned i = 1, servo_mask = LMS_FIRST_SERVO;
       i <= LMS_SERVOS_QUANTITY;
       ++i, servo_mask <<= 1)
  {
    if (selected_servos & servo_mask)
      servoWrite(i, angle);
  }
}


uint8_t LMotorShield::getMotorLastSpeed(uint8_t motor)
{
  if (1 <= motor && motor <= LMS_MOTORS_QUANTITY)
    return motors[motor - 1].last_speed;
  
  return 0;
}


LMS_Direction LMotorShield::getMotorDirection(uint8_t motor)
{
  if (1 <= motor && motor <= LMS_MOTORS_QUANTITY)
    return static_cast<LMS_Direction>(digitalRead(motors[motor - 1].dir_pin));
  
  return LMS_FORWARD;
}


bool LMotorShield::getMotorBreak(uint8_t motor)
{
  if (1 <= motor && motor <= LMS_MOTORS_QUANTITY)
    return (digitalRead(motors[motor - 1].brk_pin) == HIGH);
  
  return false;
}


uint8_t LMotorShield::getServoLastAngle(uint8_t servo)
{
  if (1 <= servo && servo <= LMS_SERVOS_QUANTITY)
    return servos[servo - 1].read();
  
  return 0;
}
