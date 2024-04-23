#include <ReefwingAHRS.h>
#include <Arduino_BMI270_BMM150.h>
#include <BasicLinearAlgebra.h>
#include <vector_type.h>
#include "Filter.h"
#include "Integral.h"

ReefwingAHRS ahrs;

Filter accelFilter, gyroFilter;
Integral velIntegral, posIntegral;
DeltaTime dt;
ZVU zvu;

SensorData data;

#define GRAVITY 9.81
#define ACCEL_THRESH 1
#define GYRO_THRESH 3.0

char input = 'G';

float step_cum = 0;

void setup() {
  ahrs.begin();

  accelFilter.begin(5, 119);
  gyroFilter.begin(5, 119);
  zvu.begin(ACCEL_THRESH, GYRO_THRESH, 15);
  
  ahrs.setFusionAlgorithm(SensorFusion::MAHONY);
  ahrs.setDeclination(-9.25059);
  ahrs.setKp(40.0);
  ahrs.setKi(0.0);

  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("BMI270 & BMM150 IMUs Not Detected.");
    while(1);
  }

  Serial.println("Ready!!!\n");

  while(1) {
    if(Serial.available()){
      input = readChar();
    }
    updateLoop();
//    printGyro();
  }
  
}

//void printGyro(){
//  imu.updateSensorData();
//  printVector("Gyro: ", {data.gx, data.gy, data.gz});
//}


void updateLoop(){
  if (IMU.gyroscopeAvailable()) {  
    IMU.readGyroscope(data.gx, data.gy, data.gz);
    data.gTimeStamp = micros();
  }
  if (IMU.accelerationAvailable()) {  
    IMU.readAcceleration(data.ax, data.ay, data.az);  
    data.aTimeStamp = micros();
  }
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(data.mx, data.my, data.mz);
    data.mTimeStamp = micros();
  }
  ahrs.setData(data); //data is a struct {ax,ay,az,gx,gy,gz,mx,my,mz,gTimeStamp,aTimeStamp,mTimeStamp}
  ahrs.update();
  vec3_t accel = getAccel(data);
  vec3_t gyro = getGyro(data);
  vec3_t accel_filtered = accelFilter.step(accel);
  vec3_t gyro_filtered = gyroFilter.step(gyro);
  vec3_t gravity = getGravity(ahrs.getQuaternion());
  vec3_t ag = projectAccelOnGravity(accel, gravity) * GRAVITY * 2 ;
  bool notMoving = zvu.check(ag, gyro);
  switch(input){
    case 'G':
      printVector("Gyro: ", gyro);
      break;
    case 'H':
      printVector("Gyro_filtered: ", gyro_filtered);
      break;
    case 'A':
      printVector("Accel: ", accel * GRAVITY);
      break;
    case 'B':
      printVector("Accel_filtered: ", accel_filtered);
      break;
    case 'N':
      printVector("Proj_Norm: ", ag, ACCEL_THRESH * notMoving);
      break;
    case 'P': {
      calculatePos(ag, data.aTimeStamp);
      printVector("AVP: ", velIntegral.prev.mag(), velIntegral.cum.mag(), posIntegral.cum.mag());
      break;
    }
    case 'R':
      resetPos();
      input = 'P';
      break;
    case 'S':
      if(notMoving){
        if(posIntegral.cum.mag() > 0.02) {
          printVector("Position: ", posIntegral.cum);
          step_cum += posIntegral.cum.mag();
          Serial.print("Cumulative dist: ");
          Serial.print(step_cum);
          Serial.print(" Total Time: ");
          Serial.print(dt.cumDiff(data.aTimeStamp));
          Serial.print(" Timestamp: ");
          Serial.println(data.aTimeStamp); // Tommy wants output: time,step_length
        }
        resetPos();
        dt.first = data.aTimeStamp;
      }
      else calculatePos(ag, data.aTimeStamp);
      break;
    case 'C':
      step_cum = 0;
      input = 'S';
      break;
//    case 'Q':
//      vec3_t ag = projectAccelOnGravity(accel_filtered, gravity);
//      calculatePos(ag, data.aTimeStamp);
//      printVector("AVP: ", velIntegral.prev.x, velIntegral.cum.x, posIntegral.cum.x);
//      break;
//    case 'S':
//      velIntegral.reset(accel, data.aTimeStamp);
//      posIntegral.reset({0, 0, 0}, data.aTimeStamp);
//      input = 'Q';
//      break;
    default:
      break;
  }
}

vec3_t getGravity(Quaternion q){
  float r0 = 2*(q.q1*q.q3-q.q0*q.q2);
  float r1 = 2*(q.q2*q.q3+q.q0*q.q1);
//  float r2 = 2*(q.q0*q.q0+q.q3*q.q3)-1;
  float r2 = q.q0*q.q0 - q.q1*q.q1 - q.q2*q.q2 + q.q3*q.q3;
  return {r0, r1, r2};
}

vec3_t projectAccelOnGravity(vec3_t a, vec3_t g){
  return a - a.dot(g)*g;
}

vec3_t subtractGravity(vec3_t a, vec3_t g){
  return a - g;
}

vec3_t calculatePos(vec3_t a, uint32_t ts){
  float delta = dt.step(data.aTimeStamp);
//  Serial.print("dt: ");
//  Serial.println(delta, 6);
  return posIntegral.step(velIntegral.step(a, delta), delta);
}

void resetPos(){
  velIntegral.reset(getAccel(data));
  posIntegral.reset({0, 0, 0});
  dt.set(data.aTimeStamp);
}

void resetVelocity() {
  velIntegral.reset(getAccel(data));
  posIntegral.resetPrev({0, 0, 0});
  dt.set(data.aTimeStamp);
}

void loop() {}
