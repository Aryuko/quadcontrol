//Imports to read from the MPU
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

//Classes for MPU
MOU6050 mpu(0x69);

//Vars for the MPU
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//SensorValues
int sensorACCL_X, sensorACCL_Y, sensorACCL_Z;  //Measured in g
int sensorGYRO_X, sensorGYRO_Y, sensorGYRO_Z;  //Measuerd in rad/s
int sensorANGL_X, sensorANGL_Y, sensorANGL_Z;  //Measured in rad

//TargetValues
int targetACCL_X, targetACCL_Y, targetACCL_Z;  //Measured in g
int targetGYRO_X, targetGYRO_Y, targetGYRO_Z;  //Measured in rad/s
int targetANGL_X, targetANGL_Y, targetANGL_Z;  //Measured in rad

//Displacment - The acceleration displacment has been smashed in the angle displacment
int rollDisplacment, pitchDisplacment, yawDisplacment;
int rollPerSecondDisplacment, pitchPerSecondDisplacment, yawPerSecondDisplacment;

void setup() {
  Serial.begin(9600);

}

void loop() {
  retriveInput();
  
  retriveSensorData();
  
  calcDisplacment();
  
  calcStrategy();
  
  executeStrategy();
}

void initSensors() {
  Wire.begin();  //Start the I2C bus
  mpu.initialize();  //Init the mpu

  if(!mpu.testConnection()) {  //Test connection to the mpu 
    Serial.println("MPU connection failed");
  }
  
  devStatus = mpu.dmpInitialize();  //Init the Digital Motion Processor
  
  //Offsets for the MPU
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(920);
  
  if(devStatus == 0) {
    mpu.setDMPEnabled(true);  //Turn on the Digital Motion Processor
    
    attachInterrupt(0, dmpDataReady, RISING);  //These one/two lines shouldn't do much, but included out of fear it may not work if not
    mpuIntStatus = mpu.getIntStatus();
    
    dmpReady = true;  //Flag the DMP as ok
    packetSize = mpu.dmpGetFIFOPacketSize();  //Get expected DMP packet size for later comparison
  }
  else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
}

void retriveInput() {
}

void retriveSensorData() {
}

void calcDisplacment() {
}

void calcStrategy() {
}

void executeStrategy() {
}
