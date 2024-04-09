#include <ReefwingAHRS.h>
#include <ReefwingLSM9DS1.h>
#include <BasicLinearAlgebra.h>
#include <Time.h> 

using namespace BLA;

ReefwingLSM9DS1 imu;
ReefwingAHRS ahrs;

#define GRAVITY 9.81
#define BUF_LENGTH 2000

//potential memory issues here oops lol 
BLA::Matrix<3> accel_array[BUF_LENGTH]; // fill this in starting at t_su until receive a t_sd 
uint32_t time_stamp_array[BUF_LENGTH]; 

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
  while (!Serial);

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

//   imu.updateSensorData();
//   ahrs.setData(imu.data);
//   ahrs.update();
//   accel_array[0] = {0,0,0}; // projec
//   time_stamp_array[0] = imu.data.aTimeStamp; 
//   i = 1; 

   Serial.println(" accel \t vel \t pos ");

}
bool printOrientation = false;
bool printRawAccel = false;

void loop() {
  imu.updateSensorData();
  ahrs.setData(imu.data); //imu.data is a struct {ax,ay,az,gx,gy,gz,mx,my,mz,gTimeStamp,aTimeStamp,mTimeStamp}
  ahrs.update();
  if(printOrientation){
    Serial.print("Orientation: ");
    Serial.print(ahrs.angles.yaw);
    Serial.print(" ");
    Serial.print(ahrs.angles.pitch);
    Serial.print(" ");
    Serial.println(ahrs.angles.roll);
  }
  if(printRawAccel){
    Serial.print("Acceleration: ");
    Serial.print(imu.data.ax);
    Serial.print(" ");
    Serial.print(imu.data.ay);
    Serial.print(" ");
    Serial.print(imu.data.az);
    Serial.print(" ");
    Serial.println(magnitude(imu.data.ax, imu.data.ay, imu.data.az));
  }
  if(!Serial.available()) return;
    switch(readChar()){
        case 'M':
          measure();
          break;
        case 'O':
          printOrientation = !printOrientation;
          break;
        case 'A':
          printRawAccel = !printRawAccel;
          break;
    }
}

float accel_thresh = 0.02;
void measure(){
    int i = 0; 
    Serial.println("Mesuring...");
    while (!Serial.available()){
        imu.updateSensorData();
        ahrs.setData(imu.data); //imu.data is a struct {ax,ay,az,gx,gy,gz,mx,my,mz,gTimeStamp,aTimeStamp,mTimeStamp}
        ahrs.update();

//        if( imu.data.aTimeStamp <= time_stamp_array[i-1] ) return;
        //don't do anything if there's no new data (aka old timestamp is same as current timestamp)

        Quaternion q = ahrs.getQuaternion();

        float r0 = 2*(q.q1*q.q3-q.q0*q.q2);
        float r1 = 2*(q.q2*q.q3+q.q0*q.q1);
//        float r2 = 2*(q.q0*q.q0+q.q3*q.q3)-1;
        float r2 = q.q0*q.q0 - q.q1*q.q1 - q.q2*q.q2 + q.q3*q.q3;


        BLA::Matrix<3> gravity = {r0, r1, r2}; // rotated gravity in device xyz coordinate basis 
        BLA::Matrix<3> accel = {imu.data.ax, imu.data.ay, imu.data.az};

        float vertical_accel = (~accel*gravity)(0); // dot product 

        for(int j = 0; j < 3; j++) {
            gravity(j) *= vertical_accel;
        }

        BLA::Matrix<3> proj = accel - gravity; // unit g's (not m/s^2)
//        accel_array[i] = proj;
        float ax_mod = abs(imu.data.ax) > accel_thresh ? imu.data.ax : 0;
        float ay_mod = abs(imu.data.ay) > accel_thresh ? imu.data.ay : 0;
        float az_mod = abs(imu.data.az) > accel_thresh ? imu.data.az : 0;
        accel_array[i] = {ax_mod, ay_mod, az_mod};
        time_stamp_array[i] = imu.data.aTimeStamp;
        if(i >= BUF_LENGTH - 1) break;
        i++;
    }
    Serial.print("Length: ");
    float len = calculate_step_length(accel_array, time_stamp_array, i);
    Serial.println(len);
    return;
}

char readChar()
{  char ch;
   ch= toupper(Serial.read());   
   delay(10);
   while (Serial.available()){Serial.read();delay(1);} // empty readbuffer 
   return ch;
}


//BLA::Matrix<3> mov_avg(BLA::Matrix<3> accel_array[], int idx){
//  float avg_x, avg_y, avg_z = 0;
//  for(int i = idx-5; i < idx+5; i++){
//    avg_x += accel_array[i](0);
//    avg_y += accel_array[i](1); 
//    avg_z += accel_array[i](2);
//  }
//  avg_x *= (GRAVITY / 10.0);
//  avg_y *= (GRAVITY / 10.0);
//  avg_z *= (GRAVITY / 10.0);
//  return {avg_x, avg_y, avg_z};
//
//}

float magnitude(float x, float y, float z){
  return sqrt(x*x + y*y + z*z);
}

float calculate_step_length(BLA::Matrix<3> accel_array[], uint32_t time_stamp_array[], int len) { //, uint32_t t_sd, uint32_t t_su){
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
  float magn_a, magn_v, magn_p = 0;
  float dt; // time between previous entry and current entry, in seconds 

//  Serial.print("polling instances: ");
//  Serial.println(len);
  int buf = 0;

  for(int i = 1 + buf; i < len - buf; i++){
//    BLA::Matrix<3> a = mov_avg(accel_array, i);
    BLA::Matrix<3> a = accel_array[i];
    BLA::Matrix<3> b = accel_array[i-1];
    dt = (float)(time_stamp_array[i] - time_stamp_array[i-1] )/ (float)1e6; //in seconds instead of us 
    ax = a(0) * GRAVITY; // multiply by gravity to get into m/s^2 rather than g's 
    ay = a(1) * GRAVITY;
    az = a(2) * GRAVITY; 
    magn_a = magnitude(ax,ay,az) - 1*GRAVITY;
    magn_v += magn_a * dt;
    magn_p += magn_v * dt; 
    float bx = b(0) * GRAVITY;
    float by = b(1) * GRAVITY;
    float bz = b(2) * GRAVITY;
    float dvx = (ax + bx) * 0.5 * dt;
    float dvy = (ay + by) * 0.5 * dt;
    float dvz = (az + bz) * 0.5 * dt;
    px += (vx + dvx * 0.5) * dt;
    py += (vy + dvy * 0.5) * dt;
    pz += (vz + dvz * 0.5) * dt;
    vx += dvx;
    vy += dvy;
    vz += dvz;
    // px += (vx + dvx * 0.5) * dt + 0.5 * ax * dt * dt;
    // py += vy * dt + 0.5 * ay * dt * dt; 
    // pz += vz * dt + 0.5 * az * dt * dt; 

//    cur_length += sqrt(px*px + py*py + pz*pz);
//    if(i == 10){
//      Serial.print("dt: ");
//      Serial.print(dt);
//      Serial.print(" increment length: ");
//      Serial.print(sqrt(px*px + py*py + pz*pz));
//      Serial.print(" cur_length: ");
//      Serial.println(cur_length);
//    }

//    Serial.print(ax);
//    Serial.print(" ");
//    Serial.print(ay);
//    Serial.print(" ");
//    Serial.println(az);
    Serial.print(magn_a);
    Serial.print(" ");
    Serial.print(magn_v);
    Serial.print(" ");
    Serial.println(magn_p);
//    Serial.print(" ");
//    Serial.print(magnitude(ax, ay, az));
//    Serial.print(" ");
//    Serial.print(magnitude(vx, vy, vz));
//    Serial.print(" ");
//    Serial.println(magnitude(px, py, pz));
  }
    return magn_p; 
//  return magnitude(px, py, pz); 

}
