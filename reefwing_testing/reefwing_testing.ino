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
#include <BasicLinearAlgebra.h>
#include <Time.h> 

using namespace BLA;

ReefwingLSM9DS1 imu;
ReefwingAHRS ahrs;

//  Display and Loop Frequency
bool readData = false;
const float readThresh = 2.5;

uint32_t dt, t_su, t_sd = 0;

float cX, cY, cZ = 0;

float prev_time = 0; 

int status;

int i, i_su, i_sd, instances_above_thresh =  0; 

int step_count; 

#define z_sd_thresh 2 // double gravity for step down threshold 
#define GRAVITY 9.81
#define BUF_LENGTH 2000

//potential memory issues here oops lol 
BLA::Matrix<3> accel_array[BUF_LENGTH]; // fill this in starting at t_su until receive a t_sd 
uint32_t time_stamp_array[BUF_LENGTH]; 
float vert_accel_array[BUF_LENGTH]; 

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

  imu.updateSensorData();
  ahrs.setData(imu.data);
  ahrs.update();
  accel_array[0] = {0,0,0}; // projec
  time_stamp_array[0] = millis(); 
  i = 1; 


  Serial.println(" aX \t aY \t aZ ");
}

void loop() {

  imu.updateSensorData();
  ahrs.setData(imu.data); //imu.data is a struct {ax,ay,az,gx,gy,gz,mx,my,mz,gTimeStamp,aTimeStamp,mTimeStamp}
  ahrs.update();

  if( imu.data.aTimeStamp <= time_stamp_array[i-1] ) return;
  //don't do anything if there's no new data (aka old timestamp is same as current timestamp)

/*
 if (millis() - previousMillis >= displayPeriod) {
   //  Display sensor data every displayPeriod, non-blocking.
   Serial.print("--> Roll: ");
   Serial.print(ahrs.angles.roll, 2);
   Serial.print("\tPitch: ");
   Serial.print(ahrs.angles.pitch, 2);
   Serial.print("\tYaw: ");
   Serial.print(ahrs.angles.yaw, 2);
   Serial.print("\tHeading: ");
   Serial.print(ahrs.angles.heading, 2);
   Serial.print("\tLoop Frequency: ");
   Serial.print(loopFrequency);
   Serial.println(" Hz");

   loopFrequency = 0;
   previousMillis = millis();
 } */


//MM 
  // int status = detectStepDown(accel_array, i, prev_check_idx); // go back some amount within the array to see if we should detect a step_down event 

//  if(millis() - previousMillis >= calculatePeriod){

//  }
/*
 Serial.print("Orientation: ");
 Serial.print(ahrs.angles.yaw);
 Serial.print(" ");
 Serial.print(ahrs.angles.pitch);
 Serial.print(" ");
 Serial.println(ahrs.angles.roll);

 Serial.print("Quaternion: ");
 Serial.print(ahrs.getQuaternion().q0);
 Serial.print(" ");
 Serial.print(ahrs.getQuaternion().q1);
 Serial.print(" ");
 Serial.print(ahrs.getQuaternion().q2);
 Serial.print(" ");
 Serial.println(ahrs.getQuaternion().q3);
*/

  Quaternion q = ahrs.getQuaternion();

  float r0 = 2*(q.q1*q.q3-q.q0*q.q2);
  float r1 = 2*(q.q2*q.q3+q.q0*q.q1);
  float r2 = 2*(q.q0*q.q0+q.q3*q.q3)-1;

//  float ax = imu.data.ax - r0;
//  float ay = imu.data.ay - r1;
//  float az = imu.data.az - r2;

  BLA::Matrix<3> gravity = {r0, r1, r2}; // rotated gravity in device xyz coordinate basis 
  BLA::Matrix<3> accel = {imu.data.ax, imu.data.ay, imu.data.az};

  float vertical_accel = (~accel*gravity)(0); // dot product 

  for(int j = 0; j < 3; j++) {
    gravity(j) *= vertical_accel;
  }

  BLA::Matrix<3> proj = accel - gravity; // unit g's (not m/s^2)



  
  //populate array, increment index 
//  prev_time = time; 
//  time = millis(); //?? 
//  dt = time - prev_time; 
  accel_array[i] = proj; // projec
  time_stamp_array[i] = imu.data.aTimeStamp; //dt; 
  vert_accel_array[i] = vertical_accel;
  i++; 


  Serial.println(vertical_accel);
  


  // vector of acceleartion in the plane normal to gravity 

//  Serial.print("Projection:\t");
//  Serial.print(proj(0));
//  Serial.print('\t');
//  Serial.print(proj(1));
//  Serial.print('\t');
//  Serial.println(proj(2));



//MM 
  //detect step-down 
  if(vertical_accel > z_sd_thresh){
    instances_above_thresh ++; 
    Serial.println("z accel threshold reached"); 
  } else {
    instances_above_thresh = 0; // not multiple instances_above_thresh in a row 
  }

  if(instances_above_thresh >= 2){
    // if(
    //   accel_array[i-2][3] < 
    // ){
    Serial.print("step down!!!\n"); 
    
    //FOR NOW: step up is the previous step-down 
    t_su = t_sd; 
    i_su = i_sd; 
    i_sd = i; 

    t_sd = millis(); 
    instances_above_thresh = 0; 
    float step_length = calculate_step_length(accel_array, time_stamp_array, i_su, i); 
    Serial.print("step length: ");
    Serial.println(step_length);
    i = 1;
    // }
  }

  //TODO: detect step-up 


  //detect step-up 

/*
 Serial.print(r0);
 Serial.print('\t');
 Serial.print(r1);
 Serial.print('\t');
 Serial.println(r2);

 Serial.print(imu.data.ax);
 Serial.print('\t');
 Serial.print(imu.data.ay);
 Serial.print('\t');
 Serial.println(imu.data.az);

 Serial.print(imu.data.ax - r0);
 Serial.print('\t');
 Serial.print(imu.data.ay - r1);
 Serial.print('\t');
 Serial.println(imu.data.az - r2);

 float aTot = abs(ay) + abs(ax) + abs(az);
 if(aTot > readThresh){
   if(readData){
     float dist = sqrt(cX*cX+cY*cY+cZ*cZ);
     Serial.print("Distance: ");
     Serial.println(dist);
    }
   readData = !readData;
  }

 if(readData){
   Serial.print(ax);
   Serial.print('\t');
   Serial.print(ay);
   Serial.print('\t');
   Serial.print(az);
   Serial.print('\t');
   
  }

 Serial.print(readThresh);
 Serial.print('\t');
 Serial.println(aTot);
*/

}

// void append_accel(float ax, float ay, float az, float dt, BLA::Matrix<3> accel_array, uint32_t time_stamp_array[]){

// }

// int detectStepDown(BLA::Matrix<4> accel_array[], int cur_idx, int prev_check_idx){

// }

float calculate_step_length(BLA::Matrix<3> accel_array[], uint32_t time_stamp_array[], int i_su, int i_sd) { //, uint32_t t_sd, uint32_t t_su){
  // float step_time = t_sd - t_su; 

  // accel_array entry has device xyz acceleration values MINUS any acceleration in the true world z (gravity) axis 
  // only acceleration in the ground xy plane, but has xyz components because it's pre-rotation (device coordinate) values 
  
  float cur_length = 0; 
  float ax, ay, az;
  float vx = 0;
  float vy = 0; 
  float vz = 0;
  float px = 0;
  float py = 0; 
  float pz = 0;
  float dt; // time between previous entry and current entry, in seconds 

  for(int i = i_su; i < i_sd; i++){
    ax = accel_array[i](0) * GRAVITY ; // multiply by gravity to get into m/s^2 rather than g's 
    ay = accel_array[i](1) * GRAVITY ;
    az = accel_array[i](2) * GRAVITY ; 
    dt = ( (float)time_stamp_array[i] - (float)time_stamp_array[i-1] )/ 1000.0; //in seconds instead of ms 
    vx += ax * dt;
    vy += ay * dt;
    vz += az * dt;
    px += vx * dt + 0.5 * ax * dt * dt;
    py += vy * dt + 0.5 * ay * dt * dt; 
    pz += vz * dt + 0.5 * az * dt * dt; 
    cur_length += sqrt(px*px + py*py + pz*pz);
  }

  return cur_length; 

}
