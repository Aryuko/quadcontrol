#include <Servo.h>
#include <Bridge.h>

Servo myServo;
int LED_PIN = 13;

void setup() {
  // put your setup code here, to run once:
  myServo.attach(9);  //Attach esc to pin 9
  
  myServo.write(180);  //Set the throttle to 0
  
  Serial.begin(9600);
}

int throttle = 0;  //Current throttle
int delta = 1;  //Amount the throttle shall be displaced each timestep

void loop() {
  Serial.write(myServo.read());
  if(Serial.available() > 0) {
    if(Serial.peek() == 'H') {
      Serial.read();
      myServo.write(180);
    }
    else if(Serial.peek() == 'L') {
      Serial.read();
      myServo.write(0);
    }
    else {
     myServo.write(Serial.parseInt());
    }
  }
    
}
