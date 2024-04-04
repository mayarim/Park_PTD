/******************************************************************
  @file       nano33BLErev1.ino
  @brief      Print roll, pitch, yaw and heading angles using the
              LSM9DS1 IMU on the Nano 33 BLE & Sense rev1
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     2.2.0
  Date:        10/02/23

  1.0.0 Original Release.                         22/02/22
  1.1.0 Added NONE fusion option.                 25/05/22
  2.0.0 Changed Repo & Branding                   15/12/22
  2.0.1 Invert Gyro Values PR                     24/12/22
  2.1.0 Updated Fusion Library                    30/12/22
  2.2.0 Add support for Nano 33 BLE Sense Rev. 2  10/02/23

  This sketch is configured to work with the MADGWICK, MAHONY,
  CLASSIC, COMPLEMENTARY, KALMAN & NONE Sensor Fusion options. Set the 
  algorithm that you wish to use with:

  ahrs.setFusionAlgorithm(SensorFusion::MADGWICK);

******************************************************************/

#include <ReefwingAHRS.h>
#include <ReefwingLSM9DS1.h>

ReefwingLSM9DS1 imu;
ReefwingAHRS ahrs;

//  Display and Loop Frequency
bool readData = false;
const float readThresh = 2.5;

float cX, cY, cZ = 0;

void setup() {
  //  Initialise the LSM9DS1 IMU & AHRS
  //  Use default fusion algo and parameters
  imu.begin();
  ahrs.begin();
  
  //  If your IMU isn't autodetected and has a mag you need
  //  to add: ahrs.setDOF(DOF::DOF_9);
  ahrs.setDOF(DOF::DOF_9);
  ahrs.setFusionAlgorithm(SensorFusion::MAHONY);
  ahrs.setDeclination(62.9252);
//  ahrs.setAlpha(0.9);
  

  //  Start Serial and wait for connection
  Serial.begin(115200);
//  while (!Serial);

  Serial.print("Detected Board - ");
  Serial.println(ahrs.getBoardTypeString());

  if (imu.connected()) {
    Serial.println("LSM9DS1 IMU Connected."); 
    Serial.println("Calibrating IMU...\n"); 
    imu.start();
    imu.calibrateGyro();
    imu.calibrateAccel();
    imu.calibrateMag();

    delay(20);
    //  Flush the first reading - this is important!
    //  Particularly after changing the configuration.
    imu.readGyro();
    imu.readAccel();
    imu.readMag();
  } 
  else {
    Serial.println("LSM9DS1 IMU Not Detected.");
    while(1);
  }
  Serial.println(" aX \t aY \t aZ ");
}

void loop() {
  imu.updateSensorData();
  ahrs.setData(imu.data);
  ahrs.update();

//  if (millis() - previousMillis >= displayPeriod) {
//    //  Display sensor data every displayPeriod, non-blocking.
//    Serial.print("--> Roll: ");
//    Serial.print(ahrs.angles.roll, 2);
//    Serial.print("\tPitch: ");
//    Serial.print(ahrs.angles.pitch, 2);
//    Serial.print("\tYaw: ");
//    Serial.print(ahrs.angles.yaw, 2);
//    Serial.print("\tHeading: ");
//    Serial.print(ahrs.angles.heading, 2);
//    Serial.print("\tLoop Frequency: ");
//    Serial.print(loopFrequency);
//    Serial.println(" Hz");
//
//    loopFrequency = 0;
//    previousMillis = millis();
//  }

  Serial.print("Orientation: ");
  Serial.print(ahrs.angles.yaw);
  Serial.print(" ");
  Serial.print(ahrs.angles.pitch);
  Serial.print(" ");
  Serial.println(ahrs.angles.roll);

//  Serial.print("Quaternion: ");
//  Serial.print(ahrs.getQuaternion().q0);
//  Serial.print(" ");
//  Serial.print(ahrs.getQuaternion().q1);
//  Serial.print(" ");
//  Serial.print(ahrs.getQuaternion().q2);
//  Serial.print(" ");
//  Serial.println(ahrs.getQuaternion().q3);

  Quaternion q = ahrs.getQuaternion();

  float r0 = 2*(q.q1*q.q3-q.q0*q.q2);
  float r1 = 2*(q.q2*q.q3+q.q0*q.q1);
  float r2 = 2*(q.q0*q.q0+q.q3*q.q3)-1;

  float ax = imu.data.ax - r0;
  float ay = imu.data.ay - r1;
  float az = imu.data.az - r2;

//  Serial.print(r0);
//  Serial.print('\t');
//  Serial.print(r1);
//  Serial.print('\t');
//  Serial.println(r2);
//
//  Serial.print(imu.data.ax);
//  Serial.print('\t');
//  Serial.print(imu.data.ay);
//  Serial.print('\t');
//  Serial.println(imu.data.az);

//  Serial.print(imu.data.ax - r0);
//  Serial.print('\t');
//  Serial.print(imu.data.ay - r1);
//  Serial.print('\t');
//  Serial.println(imu.data.az - r2);

//  float aTot = abs(ay) + abs(ax) + abs(az);
//  if(aTot > readThresh){
//    if(readData){
//      float dist = sqrt(cX*cX+cY*cY+cZ*cZ);
//      Serial.print("Distance: ");
//      Serial.println(dist);
//    }
//    readData = !readData;
//  }
//
//  if(readData){
//    Serial.print(ax);
//    Serial.print('\t');
//    Serial.print(ay);
//    Serial.print('\t');
//    Serial.print(az);
//    Serial.print('\t');
//    
//  }

//  Serial.print(readThresh);
//  Serial.print('\t');
//  Serial.println(aTot);


}
