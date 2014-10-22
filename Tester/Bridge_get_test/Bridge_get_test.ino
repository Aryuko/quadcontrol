#include <Bridge.h>

void setup() {
  // put your setup code here, to run once:
  
  Bridge.begin();
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  int bufferSize = 10;
  char myData[bufferSize];
  
  int i;
  for(i = 0; i < bufferSize; i++) {
    myData[i] = (char) 0;
  }
  Bridge.get("test", myData, bufferSize);
  
  
  for(i = 0; i < bufferSize; i++) {
    Serial.print(myData[i]);
  }
  Serial.println();
}
