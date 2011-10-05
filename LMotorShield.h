#ifndef LMOTORSHIELD_H_
#define LMOTORSHIELD_H_

#include <stdint.h>
#include <WProgram.h>
#include <Servo.h>

enum
{
  LMS_MOTORS_AMOUNT = 2,
  LMS_SERVOS_AMOUNT = 4
};

enum
{
  LMS_FIRST_MOTOR = 1 << 0,
  LMS_MOTOR1 = LMS_FIRST_MOTOR,
  LMS_MOTOR2 = LMS_FIRST_MOTOR << 1,
  LMS_MOTORS = LMS_MOTOR1 | LMS_MOTOR2,

  LMS_FIRST_SERVO = LMS_FIRST_MOTOR << LMS_MOTORS_AMOUNT,
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
  void motorBreak(uint8_t motor, bool on);
  
  void multipleMotorSpeed(unsigned selected_motors, uint8_t speed);
  void multipleMotorDirection(unsigned selected_motors, LMS_Direction direction);
  void multipleMotorBreak(unsigned selected_motors, bool on);
  
  void servoWrite(uint8_t servo, uint8_t angle);
  void multipleServoWrite(unsigned selected_servos, uint8_t angle);
  
  // TODO
  //motorStop
  //motorRun

private:
  struct Motor
  {
    uint8_t pwm_pin;
    uint8_t dir_pin;
    uint8_t brk_pin;
  };
  
  typedef Motor MotorArray[LMS_MOTORS_AMOUNT];
  MotorArray motors;
  
  Servo servos[LMS_SERVOS_AMOUNT];
  
  typedef uint8_t PinArray[LMS_SERVOS_AMOUNT];
  PinArray servo_pins;
};

#endif
