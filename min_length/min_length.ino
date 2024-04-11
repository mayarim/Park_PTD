#include <ReefwingAHRS.h>
#include <ReefwingLSM9DS1.h>
#include <vector_type.h>
#include "Filter.h"
#include "Integral.h"
#include <ArduinoBLE.h>
#include "Adafruit_DRV2605.h"

Adafruit_DRV2605 drv;

// BLE Data Service
BLEService dataService("180A");

// Left Device
BLEStringCharacteristic dataCharacteristic("0000AAAA-0000-1000-8000-00805F9B34FB", BLERead | BLENotify, 128); // 20 is the maximum length of the string
BLEFloatCharacteristic writeCharacteristic("0000BBBB-0000-1000-8000-00805F9B34FB", BLEWrite);

// Right Device
//BLEStringCharacteristic dataCharacteristic("2A57", BLERead | BLENotify, 128);
//BLEFloatCharacteristic writeCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite);


ReefwingLSM9DS1 imu;
ReefwingAHRS ahrs;
//Configuration imuConfig;

Filter accelFilter, gyroFilter;
Integral velIntegral, posIntegral;
DeltaTime dt;
ZVU zvu;

#define GRAVITY 9.81
#define ACCEL_THRESH 1
#define GYRO_THRESH 3.0
#define MIN_STEP_LENGTH 0.02
#define MAX_STEP_LENGTH 5.00
#define M_TO_CM 100

char input = 'F';

float step_cum = 0;
float step_target = 0.50;

bool connectBLE = true;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on the built-in LED
  
  imu.begin();
  ahrs.begin();

  accelFilter.begin(5, 119);
  gyroFilter.begin(5, 119);
  zvu.begin(ACCEL_THRESH, GYRO_THRESH, 30);
  
  ahrs.setFusionAlgorithm(SensorFusion::MAHONY);
  ahrs.setDeclination(-9.25059);
  ahrs.setKp(40.0);
  ahrs.setKi(0.0);
//  ahrs.setBeta(0);
//  ahrs.setAlpha(0.95);
 // Serial.begin(115200);
//  while (!Serial);

  // Init Motor Driver
  if (! drv.begin()) {
  //  Serial.println("Could not find DRV2605");
    while (1) delay(10);
  }
  
  drv.setMode(DRV2605_MODE_INTTRIG); // default, internal trigger when sending GO command
  drv.selectLibrary(1);
  drv.setWaveform(0, 84);  // ramp up medium 1, see datasheet part 11.2
  drv.setWaveform(1, 1);  // strong click 100%, see datasheet part 11.2
  drv.setWaveform(2, 0);  // end of waveforms

  // Init BLE
  if (!BLE.begin()) {
   // Serial.println("BLE initialization failed!!!");
    while (1);
  } else {
  //  BLE.setLocalName("Right Step Detector");
    BLE.setLocalName("Left Step Detector");
    BLE.setAdvertisedService(dataService);
    BLE.setAdvertisingInterval(100);
  
    // add the characteristics to the service
    dataService.addCharacteristic(dataCharacteristic);
    dataService.addCharacteristic(writeCharacteristic);
  
    // add service
    BLE.addService(dataService);
  
    // start advertising
    BLE.advertise();
 //   Serial.println("BLE Started.");
  }

  // Init IMU
  if (imu.connected()) {
    imu.setGyroODR(GODR_952Hz);
//    Serial.println("LSM9DS1 IMU Connected."); 
//    Serial.println("Calibrating IMU...\n"); 
    imu.start();
    imu.calibrateGyro();
    imu.calibrateAccel();
    imu.calibrateMag();

    delay(20);
    //  Flush the first reading - this is important!
    //  Particularly after changing the imuConfiguration.
    imu.readGyro();
    imu.readAccel();
    imu.readMag();


/***********************
 * DON'T USE imu.getConfig()
 * For some reason causes gyro to have non-zero readings when device is not moving
 */
//    imuConfig = imu.getConfig(); 
//    printConfig();

//    imu.updateSensorData();
//    resetIntegrals();
  } 
  else {
//    Serial.println("LSM9DS1 IMU Not Detected.");
    while(1);
  }

//  Serial.println("Ready!!!\n");

  while(1) {
    if(Serial.available()){
      input = readChar();
    }
//    BLEDevice central;
////    central = BLE.central();
//    if(connectBLE) bleFetch();
    updateLoop();
  }
  
}

void minUpdate(){
  imu.updateSensorData();
  ahrs.setData(imu.data); //imu.data is a struct {ax,ay,az,gx,gy,gz,mx,my,mz,gTimeStamp,aTimeStamp,mTimeStamp}
  ahrs.update();
}

void updateLoop(){
  minUpdate();
  vec3_t accel = getAccel(imu.data);
  vec3_t gyro = getGyro(imu.data);
  vec3_t accel_filtered = accelFilter.step(accel);
  vec3_t gyro_filtered = gyroFilter.step(gyro);
  vec3_t gravity = getGravity(ahrs.getQuaternion());
  vec3_t ag = projectAccelOnGravity(accel, gravity) * GRAVITY * 2 ;
  bool notMoving = zvu.check(ag, gyro);
  switch(input){
    case 'G':
      printVector("Gyro: ", gyro);
      break;
    case 'N':
      printVector("Proj_Norm: ", ag, ACCEL_THRESH * notMoving);
      break;
    case 'S': {
      if(notMoving){
        float len = posIntegral.cum.mag();
        if(len > MIN_STEP_LENGTH && len < MAX_STEP_LENGTH) {
          printVector("Position: ", posIntegral.cum);
          step_cum += len;
    //      Serial.print("Cumulative dist: ");
     //     Serial.print(step_cum);
      //    Serial.print(" Total Time: ");
     //     Serial.print(dt.cumDiff(imu.data.aTimeStamp));
      //    Serial.print(" Timestamp: ");
      //    Serial.println(imu.data.aTimeStamp); // Tommy wants output: time,step_length
          if(len < step_target){
            vibrate();
          }
          if(connectBLE) bleSend(buildDataString(imu.data.aTimeStamp, len));
        }
        resetPos();
        dt.first = imu.data.aTimeStamp;
      }
      else calculatePos(ag, imu.data.aTimeStamp);
      break;
    }
    case 'C':
      step_cum = 0;
      input = 'S';
      break;
    case 'F':{
      unsigned long start = millis();
//      Serial.println("Fetching Step Length...");
      while(1){
        unsigned long diff = millis() - start;
        if(diff >= 10000){
          break;
        }
        if(diff % 1000 < 5) {
//          Serial.print(diff / 1000);
//          Serial.println("...");
        }
        minUpdate();
        bleFetch();
      }
      digitalWrite(LED_BUILTIN, LOW);

      input = 'C';
      break;
    }
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
  float delta = dt.step(imu.data.aTimeStamp);
  return posIntegral.step(velIntegral.step(a, delta), delta);
}

void resetPos(){
  velIntegral.reset(getAccel(imu.data));
  posIntegral.reset({0, 0, 0});
  dt.set(imu.data.aTimeStamp);
}

void resetVelocity() {
  velIntegral.reset(getAccel(imu.data));
  posIntegral.resetPrev({0, 0, 0});
  dt.set(imu.data.aTimeStamp);
}

void vibrate(){
  drv.go();
}

void loop() {}
