#include <Servo.h>
#include <LMotorShield.h>

LMotorShield lms;

void setup()
{
  lms.begin(LMS_MOTORS | LMS_SERVOS);
}

void loop()
{
  lms.motorSpeed(1, 30);
  lms.motorDirection(1, LMS_FORWARD);
  lms.motorSpeed(2, 30);
  lms.motorBackward(2);
  
  lms.multipleServoWrite(LMS_SERVOS, 90);
  
  delay(2000);
  
  lms.multipleMotorSpeed(LMS_MOTORS, 60);
  
  lms.servoWrite(1, 30);
  lms.servoWrite(2, 60);
  lms.servoWrite(3, 120);
  lms.servoWrite(4, 150);
  
  delay(2000);
  
  lms.multipleMotorStop(LMS_MOTORS);
  lms.multipleServoWrite(LMS_SERVO1 | LMS_SERVO2, 0);
  lms.multipleServoWrite(LMS_SERVO3 | LMS_SERVO4, 180);
  
  delay(2000);
  
  lms.motorRun(1);
  lms.motorRun(2);
}
