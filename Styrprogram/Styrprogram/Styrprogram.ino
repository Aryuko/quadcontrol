//Imports to read from the sensors
#include <Wire.h>

#include <helper_3dmath.h>
#include <MPU6050.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050_9Axis_MotionApps41.h>

#include <AK8975.h>



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

}

void loop() {
  retriveInput();
  
  retriveSensorData();
  
  calcDisplacment();
  
  calcStrategy();
  
  executeStrategy();
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
