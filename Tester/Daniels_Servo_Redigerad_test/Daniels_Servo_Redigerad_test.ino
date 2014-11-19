#include <Servo.h>

Servo myservo;
Servo motorA, motorB, motorC, motorD;

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

void setSpeed(int speed, Servo motor)
{
  // speed is from 0 to 100 where 0 is off and 100 is max speed
  // the following maps speed values of 0-100 to angles from 0-180
  
  int angle = map(speed, 0, 100, 0, 180);
  motor.write(angle);
  
}

void setup()
{
  
  Serial.begin(9600);
  motorA.attach(5); motorB.attach(6); motorC.attach(10); motorD.attach(11);
  while(!Serial){}
  arm();
  
}

void loop()
{
  if(Serial.available() > 0) {
    int speed = Serial.parseInt();
    if(speed >= 0 && speed <= 100) {
      setSpeed(speed, motorA);
      setSpeed(speed, motorB);
      setSpeed(speed, motorC);
      setSpeed(speed, motorD);
    }
  }
}
