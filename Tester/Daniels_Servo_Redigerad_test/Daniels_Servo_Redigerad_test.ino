#include <Servo.h>

Servo myservo;
Servo motorA, motorB, motorC, motorD;
float MOTOR_MIN = 30, MOTOR_MAX = 90;

void arm()
{
  //arm speed controller, modify as necessary for your ESC
  
  Serial.println("Arming");
  setSpeed(0, motorA); setSpeed(0, motorB); setSpeed(0, motorC); setSpeed(0, motorD);
  delay(2000);
  
  setSpeed(90, motorA); setSpeed(90, motorB); setSpeed(90, motorC); setSpeed(90, motorD);
  delay(2000);
  
  Serial.println("Armed");
  setSpeed(0, motorA); setSpeed(0, motorB); setSpeed(0, motorC); setSpeed(0, motorD);
  delay(2000);
  
}

void setThrottleRange() {
  setSpeedAllMotors(MOTOR_MAX);
  delay(2500);
  setSpeedAllMotors(MOTOR_MIN);
}

void setSpeedAllMotors(float newSpeed) {
  setSpeed(newSpeed, motorA);
  setSpeed(newSpeed, motorB);
  setSpeed(newSpeed, motorC);
  setSpeed(newSpeed, motorD);
}

void setSpeed(int speed, Servo motor) {
  // speed is from 0 to 100 where 0 is off and 100 is max speed
  // the following maps speed values of 0-100 to angles from 0-180
  
  if(speed < MOTOR_MIN) {
    speed = MOTOR_MIN;
  }
  else if(speed > MOTOR_MAX) {
    speed = MOTOR_MAX;
  }
  
  int angle = map(speed, 0, 100, 0, 180);
  motor.write(angle);
  
}

void setup()
{
  
  Serial.begin(9600);
  motorA.attach(5); motorB.attach(6); motorC.attach(10); motorD.attach(11);
  setSpeed(MOTOR_MIN, motorA); setSpeed(MOTOR_MIN, motorB); setSpeed(MOTOR_MIN, motorC); setSpeed(MOTOR_MIN, motorD);
  setThrottleRange();
}

void loop()
{
  if(Serial.available() > 0) {
    char sign = Serial.read();
    float newSpeed = Serial.parseFloat();
    if(sign == 'a') {
      setSpeed(newSpeed, motorA);
    }
    else if(sign == 'b') {
      setSpeed(newSpeed, motorB);
    }
    else if(sign == 'c') {
      setSpeed(newSpeed, motorC);
    }
    else if(sign == 'd') {
      setSpeed(newSpeed, motorD);
    }
  }
}
