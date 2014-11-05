//SensorValues
int sensorACCL_X, sensorACCL_Y, sensorACCL_Z;  //Measured in g
int sensorGYRO_X, sensorGYRO_Y, sensorGYRO_Z;  //Measuerd in rad/s
int sensorCMPS_X, sensorCMPS_Y, sensorCMPS_Z;  //Measured in rad

//TargetValues
int targetACCL_X, targetACCL_Y, targetACCL_Z;  //Measured in g
int targetGYRO_X, targetGYRO_Y, targetGYRO_Z;  //Measured in rad/s
int targetCMPS_X, targetCMPS_Y, targetCMPS_Z;  //Measured in rad

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
