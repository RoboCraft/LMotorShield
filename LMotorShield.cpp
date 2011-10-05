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
  
  for (unsigned i = 0; i < LMS_MOTORS_AMOUNT; ++i)
    motors[i] = motors_initial_config[i];
  
  static const PinArray servo_pins_initial_config = { 10, 9, 6, 5 };
  
  for (unsigned i = 0; i < LMS_SERVOS_AMOUNT; ++i)
    servo_pins[i] = servo_pins_initial_config[i];
}


LMotorShield::~LMotorShield()
{
}


void LMotorShield::remapMotorPins(uint8_t motor_num,
  uint8_t pwm_pin, uint8_t dir_pin, uint8_t brk_pin)
{
  if (1 < motor_num && motor_num <= LMS_MOTORS_AMOUNT)
  {
    Motor &m = motors[motor_num - 1];
    
    m.pwm_pin = pwm_pin;
    m.dir_pin = dir_pin;
    m.brk_pin = brk_pin;
  }
}


void LMotorShield::remapServoPins(uint8_t servo_num, uint8_t signal_pin)
{
  if (1 <= servo_num && servo_num <= LMS_SERVOS_AMOUNT)
    servo_pins[servo_num - 1] = signal_pin;
}


void LMotorShield::begin(unsigned selected_units)
{
  for (unsigned i = 0, motor_mask = LMS_FIRST_MOTOR;
       i < LMS_MOTORS_AMOUNT;
       ++i, motor_mask <<= 1)
  {
    if (selected_units & motor_mask)
    {
      pinMode(motors[i].dir_pin, OUTPUT);
      pinMode(motors[i].brk_pin, OUTPUT);
    }
  }
  
  for (unsigned i = 0, servo_mask = LMS_FIRST_SERVO;
       i < LMS_SERVOS_AMOUNT;
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
  if (1 <= motor && motor <= LMS_MOTORS_AMOUNT) 
    analogWrite(motors[motor - 1].pwm_pin, speed);
}


void LMotorShield::motorDirection(uint8_t motor, LMS_Direction direction)
{
  if (1 <= motor && motor <= LMS_MOTORS_AMOUNT) 
    digitalWrite(motors[motor - 1].dir_pin, direction);
}


void LMotorShield::motorBreak(uint8_t motor, bool on)
{
  if (1 <= motor && motor <= LMS_MOTORS_AMOUNT)
    digitalWrite(motors[motor - 1].brk_pin, (on ? HIGH : LOW));
}


void LMotorShield::multipleMotorSpeed(unsigned selected_motors, uint8_t speed)
{
  for (unsigned i = 1, motor_mask = LMS_FIRST_MOTOR;
       i <= LMS_MOTORS_AMOUNT;
       ++i, motor_mask <<= 1)
  {
    if (selected_motors & motor_mask)
      motorSpeed(i, speed);
  }
}


void LMotorShield::multipleMotorDirection(unsigned selected_motors, LMS_Direction direction)
{
  for (unsigned i = 1, motor_mask = LMS_FIRST_MOTOR;
       i <= LMS_MOTORS_AMOUNT;
       ++i, motor_mask <<= 1)
  {
    if (selected_motors & motor_mask)
      motorDirection(i, direction);
  }
}


void LMotorShield::multipleMotorBreak(unsigned selected_motors, bool on)
{
  for (unsigned i = 1, motor_mask = LMS_FIRST_MOTOR;
       i <= LMS_MOTORS_AMOUNT;
       ++i, motor_mask <<= 1)
  {
    if (selected_motors & motor_mask)
      motorBreak(i, on);
  }
}


void LMotorShield::servoWrite(uint8_t servo, uint8_t angle)
{
  if (1 <= servo && servo <= LMS_SERVOS_AMOUNT)
    servos[servo - 1].write(angle);
}


void LMotorShield::multipleServoWrite(unsigned selected_servos, uint8_t angle)
{
  for (unsigned i = 1, servo_mask = LMS_FIRST_SERVO;
       i <= LMS_SERVOS_AMOUNT;
       ++i, servo_mask <<= 1)
  {
    if (selected_servos & servo_mask)
      servoWrite(i, angle);
  }
}
