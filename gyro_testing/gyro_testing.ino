/*
  Arduino LSM9DS1 - Simple Gyroscope
  Extended with library V2.0 function calls

  This example reads the gyroscope values from the LSM9DS1
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Nano 33 BLE Sense
  - or Arduino Uno connected to LSM9DS1 breakout board

  created 10 Jul 2019
  by Riccardo Rizzo
  
  Modified by Femme Verbeek 14 jul 2020
  
  This example code is in the public domain.
*/

#include <Arduino_LSM9DS1.h>
boolean viewInSerialPlotter=true;      // true optimises for serial plotter, false for serial monitor

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin())
  { Serial.println("Failed to initialize IMU!");
    while (1);
  } 
/*******   The gyroscope needs to be calibrated. Offset controls drift and Slope scales the measured rotation angle  *********
*****************   Copy/Replace the lines below by the output of the DIY_Calibration_Gyroscope sketch   ********************/
   IMU.setGyroFS(2);
   IMU.setGyroODR(5);
//   IMU.setGyroOffset (3.615631, -0.771332, 1.388794);
//   IMU.setGyroSlope (1.193819, 1.141039, 1.154469);
  IMU.setGyroOffset (3.781133, -0.807935, 1.357744);
  IMU.setGyroSlope (1.188142, 1.130307, 1.155754);


/*****************************************************************************************************************************     
*********  FS  Full Scale       setting 0: ±245°/s | 1: ±500°/s | 2: ±1000°/s | 3: ±2000°/s       ****************************
*********  ODR Output Data Rate setting 0:off | 1:10Hz | 2:50Hz | 3:119Hz | 4:238Hz | 5:476Hz, (not working 6:952Hz)   *******
*****************************************************************************************************************************/     
   IMU.gyroUnit= DEGREEPERSECOND;  //   DEGREEPERSECOND  RADIANSPERSECOND  REVSPERMINUTE  REVSPERSECOND  
   if (!viewInSerialPlotter)
   {  Serial.println("Gyroscope in degrees/second \n");
      Serial.print("Gyroscope Full Scale = ±");
      Serial.print(IMU.getGyroFS());
      Serial.println ("°/s");
      Serial.print("Gyroscope sample rate = ");
      Serial.print(IMU.getGyroODR());        //alias  IMU.gyroscopeSampleRate());
      Serial.println(" Hz");
      delay(4000);
   }  
   Serial.println("X \t Y \t Z");
}

void loop() {
  float x, y, z;

  if (IMU.gyroAvailable())    // alias IMU.gyroscopeAvailable
  {  
     IMU.readGyro(x, y, z);   // alias IMU.readGyroscope
     Serial.print(x);
     Serial.print('\t');
     Serial.print(y);
     Serial.print('\t');
     Serial.println(z);
  }
}
