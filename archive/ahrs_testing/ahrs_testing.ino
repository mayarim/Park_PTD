// based off of Tom Igoe's version here : https://itp.nyu.edu/physcomp/lessons/accelerometers-gyros-and-imus-the-basics

#include <Arduino_LSM9DS1.h>
#include "MadgwickAHRS.h"
#include <MahonyAHRS.h>

// initialize a Madgwick filter:
 Madgwick filter;
//Mahony filter;

const float sensorRate = 460.00; // In Hertz

void setup() {
  Serial.begin(115200);
  // attempt to start the IMU:
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    // stop here if you can't access the IMU:
    while (true);
  }
  IMU.setGyroFS(2);
  IMU.setGyroODR(5); // 476Hz
//  IMU.setGyroOffset (3.615631, -0.771332, 1.388794);
//  IMU.setGyroSlope (1.193819, 1.141039, 1.154469);   
  IMU.setGyroOffset (3.781133, -0.807935, 1.357744);
  IMU.setGyroSlope (1.188142, 1.130307, 1.155754);
  IMU.gyroUnit= DEGREEPERSECOND;

  IMU.setAccelFS(3);
  IMU.setAccelODR(5); // 476Hz
  IMU.setAccelOffset(-0.016153, -0.025595, -0.027484);
  IMU.setAccelSlope (1.003992, 0.995510, 1.002365);
  IMU.accelUnit=  GRAVITY;

  IMU.setMagnetFS(0);
  IMU.setMagnetODR(8);
  IMU.setMagnetOffset(0.477295, 8.202515, -2.402344);
  IMU.setMagnetSlope (1.132609, 1.163721, 0.989470);
  IMU.magnetUnit = MICROTESLA;
  
  // start the filter to run at the sample rate:
  filter.begin(sensorRate);
}

void loop() {
  // values for acceleration and rotation:
  float xAcc, yAcc, zAcc;
  float xGyro, yGyro, zGyro;
//  float xMag, yMag, zMag;

  // values for orientation:
  float roll, pitch, heading;
  // check if the IMU is ready to read:
  if (IMU.accelAvailable()
      && IMU.gyroAvailable()
//      && IMU.magnetAvailable()
      ) {
    // read accelerometer &and gyrometer:
    IMU.readAccel(xAcc, yAcc, zAcc);
//    IMU.readRawAccel(xAcc, yAcc, zAcc);
    IMU.readGyro(xGyro, yGyro, zGyro);
//    IMU.readRawGyro(xGyro, yGyro, zGyro);
//    IMU.readMagnet(xMag, yMag, zMag);
//    IMU.readRawMagnet(xMag, yMag, zMag);

    // update the filter, which computes orientation:
    filter.updateIMU(-xGyro, -yGyro, -zGyro, -xAcc, -yAcc, -zAcc);
//    filter.update(xGyro, yGyro, zGyro, xAcc, yAcc, zAcc, xMag, yMag, zMag);
//    filter.update(xGyro, yGyro, zGyro, xAcc, yAcc, zAcc, -yMag, -xMag, zMag);


    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);
  }
}
