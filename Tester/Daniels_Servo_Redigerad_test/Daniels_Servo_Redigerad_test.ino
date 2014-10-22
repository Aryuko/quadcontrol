#include <Servo.h>

Servo myservo;

void arm()
{
  //arm speed controller, modify as necessary for your ESC
  
  Serial.println("Arming");
  setSpeed(0);
  delay(2000);
  
  setSpeed(90);
  delay(2000);
  
  Serial.println("Armed");
  setSpeed(0);
  delay(2000);
  
}

void setSpeed(int speed)
{
  // speed is from 0 to 100 where 0 is off and 100 is max speed
  // the following maps speed values of 0-100 to angles from 0-180
  
  int angle = map(speed, 0, 100, 0, 180);
  myservo.write(angle);
  
}

void setup()
{
  
  Serial.begin(115200);
  myservo.attach(9);
  arm();
  
}

void loop()
{
  if(Serial.available() > 0) {
    int speed = Serial.parseInt();
    if(speed >= 0 && speed <= 100) {
      setSpeed(speed);
    }
  }
}
