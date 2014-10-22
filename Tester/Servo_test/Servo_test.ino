#include <Servo.h>
#include <Bridge.h>

Servo myServo;

void setup() {
  // put your setup code here, to run once:
  myServo.attach(9);  //Attach esc to pin 9
  
  myServo.write(0);  //Set the throttle to 0
  
  delay(2000);  //Wait for esc to confirm startup is complete, timting should be tuned
}

int throttle = 0;
int delta = 1;

void loop() {
  // put your main code here, to run repeatedly:
  throttle += delta;
  
  if(throttle % 180 == 0) {
    delta *= -1;
  }
  
  delay(10);
}
