  //Imports to read from the MPU
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <Servo.h>

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
float yprTarget[3];
float totThrottle = 30;

//Displacment - The acceleration displacment has been smashed in the angle displacment
float yprDisplacment[3];

const int pi = 3.1415926 * 10000000;

float MOTOR_MAX = 90, MOTOR_MIN = 30;
Servo motorA, motorB, motorC, motorD;
bool armed = false;

//PID
//YawPitchRoll - array order
float P_VALUE[] = {0, -0.02, 0.02};
float I_VALUE[] = {0, -0.00003, 0.00003};
float D_VALUE[] = {0, 2, -2};

float lastError[] = {0, 0, 0};
float Amotor, Bmotor, Cmotor, Dmotor;  //Thrust for motors

int numberOfPastError = 30;
float pastErrors[3][30] = { {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
int pastErrorIndex[3] = {0, 0, 0};
float reset[3];

int TARGET_AXIS = 1;

void setup() {
  Serial.begin(9600);
  setupMotors();
  initMPU();
}
int counter = 0;

void loop() {
  
  retriveInput();
  
  retriveSensorData();
  
  calcDisplacment();
  
  calcStrategy();
  
  executeStrategy();
  
  int cutoff = 100;
  if(counter++ > cutoff) {
    counter -= cutoff;
    
    printMPU();
    
    printPID();
  
    printDisplacment();
    
    printADJ();
    
    printAVG();
    
    printPastErrors();
    
    Serial.print("Trhottle"); Serial.println(totThrottle);
  
    printStrategy();
  }
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
  if(Serial.available() > 0) {
    char sign = Serial.read();
    float newSpeed = Serial.parseFloat();
    if(sign == 'p') {
      P_VALUE[TARGET_AXIS] = newSpeed;
    }
    else if(sign == 'i') {
      I_VALUE[TARGET_AXIS] = newSpeed;
    }
    else if(sign == 'd') {
      D_VALUE[TARGET_AXIS] = newSpeed;
    }
    else if(sign == 'a') {
      armed = !armed;
    }
    else if(sign == 's') {
      totThrottle = newSpeed;
    }
    else if(sign == 'r') {
      reset[TARGET_AXIS] = 0.0;
    }
    else if(sign == 'n') {
      numberOfPastError = newSpeed;
    }
    else if(sign == 't') {
      yprTarget[TARGET_AXIS] = newSpeed;
    }
  }
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

void printMPU() {
  Serial.print("ypr");
  Serial.print("\t\t");
  Serial.print(sensorANGL_Y);
  Serial.print("\t");
  Serial.print(sensorANGL_P);
  Serial.print("\t");
  Serial.print(sensorANGL_R);
  Serial.println();
}

void calcDisplacment() {
  int deltaV = (yprTarget[0] - sensorANGL_Y)*10000000;
  float v1 = (deltaV % pi)/10000000;
  float v2 = 2*pi - v1;
  float v;
  
  if(v1 < v2) {
    v = v2;
  }
  else {
    v = v1;
  }
  
  yprDisplacment[0] = v;

  yprDisplacment[1] = yprTarget[1] - sensorANGL_P;
  yprDisplacment[2] = yprTarget[2] - sensorANGL_R;
  
  for(int i = 0; i < 3; i++) {
    if(++pastErrorIndex[i] >= numberOfPastError) {
      pastErrorIndex[i] = 0;
    }
    
    pastErrors[i][pastErrorIndex[i]] = yprDisplacment[i];
  }
}

void printPastErrors() {
  for(int i = 0; i < 3; i++) {
    Serial.print(i); Serial.print(" avg nums");
    for(int j = 0; j < numberOfPastError; j++) {
      Serial.print("\t");
      Serial.print(pastErrors[i][j]);
    }
    Serial.println();
  }
}

void printDisplacment() {
  Serial.print("displacment");
  for(int i = 0; i < 3; i++) {
    Serial.print("\t");
    Serial.print(yprDisplacment[i]);
  }
  Serial.println();
}


float adjustment[3];
float avgPastErrors[3];
void calcStrategy() {
  //PID magic
  //P values for all axes
  
  for(int i = 0; i < 3; i++) {
    float sum = 0;
    for(int j = 0; j < numberOfPastError; j++) {
      sum += pastErrors[i][j];
    }
    avgPastErrors[i] = sum / numberOfPastError;
  }
  
  float P[3];
  for(int i = 0; i < 3; i++) {
    P[i] = P_VALUE[i] * avgPastErrors[i];
  }
  
  //I vlaues for all axes
  float I[3];
  for(int i = 0; i < 3; i++) {
    reset[i] = reset[i] + avgPastErrors[i];
    I[i] = I_VALUE[i] * reset[i];
  }
  
  //D values for all axes
  float D[3];
  for(int i = 0; i < 3; i++) {
    D[i] = D_VALUE[i] * (lastError[i] - avgPastErrors[i]);
    lastError[i] = avgPastErrors[i];
  }
  
  //Combined values for all axes
  for(int i = 0; i < 3; i++) {
    adjustment[i] = P[i] + I[i] + D[i];
  }
  
  //Distribute thrust according to adjustments
  float sharedThrottle = totThrottle * 4;
  
  //Split evenly between motorpairs and then adjust
  float AC = sharedThrottle/2.0;
  float BD = AC - adjustment[0];
  AC = AC + adjustment[0];
  
  //Split evenly in motorpairs and then adjust
  Amotor = AC / 2.0;
  Cmotor = Amotor - adjustment[1];
  Amotor = Amotor + adjustment[1];
  
  Bmotor = BD / 2.0;
  Dmotor = Bmotor - adjustment[2];
  Bmotor = Bmotor + adjustment[2];
}

void printAVG() {
  Serial.print("avg");
  for(int i = 0; i < 3; i++) {
    Serial.print("\t");
    Serial.print(avgPastErrors[i]);
  }
  Serial.println();
}

void printADJ() {
  Serial.print("adj");
  for(int i = 0; i < 3; i++) {
    Serial.print("\t");
    Serial.print(adjustment[i]);
  }
  Serial.println();
}

/* Alternate solution to thrust distribution
float distributeThrottle(float throttle, float adj[]) {
  float sharedThrottle = throttle * 4;
  
  //Split evenly between motorpairs and then adjust
  float AC = sharedThrottle/2.0;
  float BD = AC - adj[0];
  AC = AC + adj[0];
  
  //Split evenly in motorpairs and then adjust
  Amotor = AC / 2.0;
  Cmotor = Amotor - adj[1];
  Amotor = Amotor + adj[1];
  
  Bmotor = BD / 2.0;
  Dmotor = Bmotor - adj[2];
  Bmotor = Bmotor + adj[2];
  
  //Shift middlepoint in pitch roll
  float Aoffset = calcOffset(Amotor);
  float Coffset = calcOffset(Cmotor);
  if(Aoffset * Coffset > 0) {
    float amount = abs(Aoffset) < abs(Coffset) ? Aoffset : Coffset;
    Amotor -= amount;
    Cmotor -= amount;
  }
  float Boffset = calcOffset(Bmotor);
  float Doffset = calcOffset(Dmotor);
  if(Boffset * Doffset > 0) {
    float amount = abs(Boffset) < abs(Doffset) ? Boffset : Doffset;
    Bmotor -= amount;
    Dmotor -= amount;
  }
  
  //Shift middlepoint yaw
  Aoffset = calcOffset(Amotor);
  Coffset = calcOffset(Cmotor);
  Boffset = calcOffset(Bmotor);
  Doffset = calcOffset(Dmotor);
  
  
  
}

float calcOffset(float motor) {
  if(motor < MOTOR_MIN) {
    return motor - MOTOR_MIN;
  }
  else if(motor > MOTOR_MAX) {
    return MOTOR_MAX - motor;
  }
  else {
    return 0;
  }
}*/

void printStrategy() {
  Serial.println("----------");
  Serial.print("\t");      Serial.print(Bmotor);    Serial.println();
  Serial.print(Amotor);    Serial.print("\t\t");    Serial.println(Cmotor);
  Serial.print("\t");      Serial.print(Dmotor);    Serial.println();
  Serial.println("----------");
  Serial.println();
}

void jesperKrasharQuad() {
  
}

void executeStrategy() {
  setSpeed(Amotor, motorA);
  setSpeed(Bmotor, motorB);
  setSpeed(Cmotor, motorC);
  setSpeed(Dmotor, motorD);
}

void arm() {
  //arm speed controller, modify as necessary for your ESC
  
  Serial.println("Arming");
  //setSpeed(0, motorA); setSpeed(0, motorB); setSpeed(0, motorC); setSpeed(0, motorD);
  //delay(2000);
  
  setSpeed(90, motorA); setSpeed(90, motorB); setSpeed(90, motorC); setSpeed(90, motorD);
  delay(2500);
  
  Serial.println("Armed");
  setSpeed(0, motorA); setSpeed(0, motorB); setSpeed(0, motorC); setSpeed(0, motorD);
  delay(2000);
  
}

void setSpeed(int speed, Servo motor) {
  // speed is from 0 to 100 where 0 is off and 100 is max speed
  // the following maps speed values of 0-100 to angles from 0-180
  
  if(speed < MOTOR_MIN) {
    speed = MOTOR_MIN;
  }
  else if(speed > MOTOR_MAX) {
    speed = MOTOR_MAX;
  }
  
  if(!armed) {
    speed = 25;
  }
  
  int angle = map(speed, 0, 100, 0, 180);
  motor.write(angle);
  
}

void setThrottleRange() {
  setSpeedAllMotors(MOTOR_MAX);
  delay(2500);
  setSpeedAllMotors(MOTOR_MIN);
}

void setSpeedAllMotors(float newSpeed) {
  setSpeed(newSpeed, motorA);
  setSpeed(newSpeed, motorB);
  setSpeed(newSpeed, motorC);
  setSpeed(newSpeed, motorD);
}

void printPID() {
  Serial.print("PID\t");
  Serial.print(P_VALUE[TARGET_AXIS] * 1000000000); Serial.print("\t");
  Serial.print(I_VALUE[TARGET_AXIS] * 1000000000); Serial.print("\t");
  Serial.println(D_VALUE[TARGET_AXIS] * 1000000000);
  Serial.print("reset - "); Serial.println(reset[TARGET_AXIS]);
}

void setupMotors() {
  motorA.attach(5); motorB.attach(6); motorC.attach(10); motorD.attach(11);
  setSpeed(MOTOR_MIN, motorA); setSpeed(MOTOR_MIN, motorB); setSpeed(MOTOR_MIN, motorC); setSpeed(MOTOR_MIN, motorD);
  setThrottleRange();
}

void dmpDataReady() {
    mpuInterrupt = true;
}
