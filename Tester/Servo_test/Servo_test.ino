#include <Servo.h>
#include <Bridge.h>

Servo myServo;

void setup() {
  // put your setup code here, to run once:
  myServo.attach(9);  //Attach esc to pin 9
  
  myServo.write(0);  //Set the throttle to 0
  
  delay(2000);  //Wait for esc to confirm startup is complete, timting should be tuned
}

int throttle = 0;  //Current throttle
int delta = 1;  //Amount the throttle shall be displaced each timestep

void loop() {
  // put your main code here, to run repeatedly:
  throttle += delta;  //Change the throttle by delta
  
  if(throttle % 180 == 0) {  //Control if delta should be reversed, it should if throttle has reached the limit of it's definition (0 <= throttle <= 180)
    delta *= -1;
  }
  
  delay(10);  //Wait a small timestep before changing the throttle once more
}
