//Imports to read from the MPU
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

//Classes for MPU
MPU6050 mpu(0x69);

//Vars for the MPU
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

//SensorValues
double sensorACCL_X, sensorACCL_Y, sensorACCL_Z;  //Measured in g
double sensorANGL_Y, sensorANGL_P, sensorANGL_R;  //Measured in degree

//TargetValues
int targetACCL_X, targetACCL_Y, targetACCL_Z;  //Measured in g
int targetGYRO_X, targetGYRO_Y, targetGYRO_Z;  //Measured in rad/s
int targetANGL_X, targetANGL_Y, targetANGL_Z;  //Measured in rad

//Displacment - The acceleration displacment has been smashed in the angle displacment
int rollDisplacment, pitchDisplacment, yawDisplacment;
int rollPerSecondDisplacment, pitchPerSecondDisplacment, yawPerSecondDisplacment;

void setup() {
  Serial.begin(9600);
  
  while(!Serial){}
  
  initMPU();
}

void loop() {
  retriveInput();
  
  retriveSensorData();
  
  calcDisplacment();
  
  calcStrategy();
  
  executeStrategy();
}

void initMPU() {
  
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
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));

  
  }
  else if (mpuIntStatus & 0x02) {  // otherwise, check for DMP data ready interrupt (this should happen frequently)
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
      
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    
    sensorACCL_X = aaReal.x / 16384.0;
    sensorACCL_Y = aaReal.y / 16384.0;
    sensorACCL_Z = aaReal.z / 16384.0;  //Measured in g
    
    sensorANGL_Y = ypr[0] * 180/M_PI;
    sensorANGL_P = ypr[1] * 180/M_PI;
    sensorANGL_R = ypr[2] * 180/M_PI;  //Measured in degree
  }
}

void calcDisplacment() {
}

void calcStrategy() {
}

void jesperKrasharQuad{
  
}

void executeStrategy() {
}

void dmpDataReady() {
    mpuInterrupt = true;
}
