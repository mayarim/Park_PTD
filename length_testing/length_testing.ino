#include <ReefwingAHRS.h>
#include <ReefwingLSM9DS1.h>
#include <BasicLinearAlgebra.h>
#include <vector_type.h>
#include <Time.h> 

using namespace BLA;

ReefwingLSM9DS1 imu;
ReefwingAHRS ahrs;
Configuration imuConfig;

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
//  ahrs.setDOF(DOF::DOF_9);
  ahrs.setFusionAlgorithm(SensorFusion::MAHONY);
  ahrs.setDeclination(-9.25059);
//  ahrs.setAlpha(0.9);
  

  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  Serial.print("Detected Board - ");
  Serial.println(ahrs.getBoardTypeString());

  if (imu.connected()) {
//    imu.setGyroODR(GODR_952Hz);
    
    Serial.println("LSM9DS1 IMU Connected."); 
    Serial.println("Calibrating IMU...\n"); 
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

    imuConfig = imu.getConfig();
    printConfig();
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
bool printFilterAccel = false;
bool printRawGyro = false;
bool printCalc = false;

vec3_t p_accel = {0,0,0};
vec3_t p_raw_accel = {0,0,0};
uint32_t prev_ts = 0;
vec3_t vel = {0,0,0};
vec3_t pos = {0,0,0};

int fc = 5;



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
  if(printFilterAccel){
    vec3_t a = lowpass_filter({imu.data.ax, imu.data.ay, imu.data.az}, p_raw_accel, fc, 119);
    Serial.print("Filtered Acceleration: ");
    Serial.print(a.x);
    Serial.print(" ");
    Serial.print(a.y);
    Serial.print(" ");
    Serial.print(a.z);
    Serial.print(" ");
    Serial.println(a.mag());
    p_raw_accel = a;
  }
  if(printRawGyro){
    Serial.print("Gyro: ");
    Serial.print(imu.data.gx);
    Serial.print(" ");
    Serial.print(imu.data.gy);
    Serial.print(" ");
    Serial.print(imu.data.gz);
    Serial.print(" ");
    Serial.println(magnitude(imu.data.gx, imu.data.gy, imu.data.gz));
  }
  if(printCalc) {
    Quaternion q = ahrs.getQuaternion();

    float r0 = 2*(q.q1*q.q3-q.q0*q.q2);
    float r1 = 2*(q.q2*q.q3+q.q0*q.q1);
    float r2 = (q.q0*q.q0 - q.q1*q.q1 - q.q2*q.q2 + q.q3*q.q3);
    float dt = (float)(imu.data.aTimeStamp - prev_ts)/ (float)1e6; //in seconds instead of us 
    vec3_t gravity = {r0, r1, r2};
//    vec3_t accel = {imu.data.ax, imu.data.ay, imu.data.az};
    vec3_t accel = lowpass_filter({imu.data.ax, imu.data.ay, imu.data.az}, p_raw_accel, fc, 119);
    p_raw_accel = accel;
    

    vec3_t sub_grav = (accel - gravity)*GRAVITY;
    vec3_t proj_accel = (accel - accel.dot(gravity)*gravity) * GRAVITY;

    vec3_t a = sub_grav;

    vec3_t dv = (a+p_accel)*0.5*dt;
    pos += (vel + dv * 0.5) * dt;
    vel += dv;
//    a = lowpass_filter(a, p_accel, 1, 119);
    p_accel = a;

    vec3_t gyro = {imu.data.gx, imu.data.gy, imu.data.gz};
//    
//    Serial.print("Accel: ");
//    Serial.print(a.x);
//    Serial.print(" ");
//    Serial.print(a.y);
//    Serial.print(" ");
//    Serial.println(a.z);
//    
//    Serial.print("Gyro: ");
//    Serial.print(gyro.x);
//    Serial.print(" ");
//    Serial.print(gyro.y);
//    Serial.print(" ");
//    Serial.println(gyro.z);
//    Serial.print(sub_grav.mag());
//    Serial.print(" ");
//    Serial.print(proj_accel.mag());
//    Serial.print(" ");
//    Serial.print(dt);
//    Serial.print(" ");
//    Serial.println((sub_grav - proj_accel).mag());
      Serial.print(a.x);
      Serial.print(" ");
      Serial.print(vel.x);
      Serial.print(" ");
      Serial.print(pos.x);
      Serial.print(" ");
      Serial.println(isMoving(a, gyro) * 10);
  }
  prev_ts = imu.data.aTimeStamp;
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
        case 'G':
          printRawGyro = !printRawGyro;
          break;
        case 'C':
          printCalc = !printCalc;
          break;
        case 'F':
          printFilterAccel = !printFilterAccel;
          break;
        case 'S':
          readAnswer("Frequency Cutoff value ", fc);
          break;
        case 'R':
          p_accel = {0,0,0};
          vel = {0,0,0};
          pos = {0,0,0};
          break;
          
    }
}

#define ACCEL_THRESH 0.35
#define GYRO_THRESH 3.0
#define SAMPLE_THRESH 10
float accel_thresh = 0.02;
int zero_samples = 0;
bool isMoving(vec3_t a, vec3_t g){
  if(abs(a.x) <= ACCEL_THRESH &&
      abs(a.y) <= ACCEL_THRESH && 
      abs(a.z) <= ACCEL_THRESH
    ){
      Serial.println("accel thresh low");
      if(abs(g.x) <= GYRO_THRESH && 
      abs(g.y) <= GYRO_THRESH && 
      abs(g.z) <= GYRO_THRESH
      ){
        Serial.println("gyro thresh low");
        zero_samples++;
      }
    }else{
//      Serial.println("moving");
      zero_samples = 0;
    }
    return zero_samples >= SAMPLE_THRESH;
}
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
        accel_array[i] = proj;
//        float ax_mod = abs(imu.data.ax) > accel_thresh ? imu.data.ax : 0;
//        float ay_mod = abs(imu.data.ay) > accel_thresh ? imu.data.ay : 0;
//        float az_mod = abs(imu.data.az) > accel_thresh ? imu.data.az : 0;
//        accel_array[i] = {ax_mod, ay_mod, az_mod};
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

vec3_t lowpass_filter(vec3_t v, vec3_t prev, float fc, float fs){
  float tau = 1/(2.0*PI*fc);
  float alpha = (1.0/fs) / (tau + (1.0/fs));
  
//  output[0] = alpha*input[0];
  return prev + alpha*(v - prev);
  
}

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
//    magn_a = magnitude(ax,ay,az) - 1*GRAVITY;
//    magn_v += magn_a * dt;
//    magn_p += magn_v * dt; 
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
//    Serial.print(magn_a);
//    Serial.print(" ");
//    Serial.print(magn_v);
//    Serial.print(" ");
//    Serial.println(magn_p);
    Serial.print(" ");
    Serial.print(magnitude(ax, ay, az));
    Serial.print(" ");
    Serial.print(magnitude(vx, vy, vz));
    Serial.print(" ");
    Serial.println(magnitude(px, py, pz));
  }
//    return magn_p; 
  return magnitude(px, py, pz); 

}

void printConfig() {
  Serial.println("\n  ========  IMU Configuration    ========");
  Serial.print("Gyro/Accel FIFO Enabled: "); Serial.println(imuConfig.gyroAccelFIFOEnabled ? "True" : "False");
  Serial.print("FIFO Mode (0 = BYPASS, 1 = FIFO_THS, 3 = CONT_TO_FIFO, 4 = BYPASS_TO_CONT, 6 = FIFO_CONT): "); 
  Serial.println(imuConfig.fifoMode);
  Serial.print("FIFO Threshold: "); Serial.println(imuConfig.fifoThreshold);
  Serial.print("Gyro/Accel Operating Mode (0 = OFF, 1 = ACCEL, 2 = NORMAL, 3 = LOW_PWR): "); 
  Serial.println(imuConfig.gyroAccelOpMode);

  Serial.println("\n  ========  GYRO Configuration    ========");
  Serial.print("Power Down: "); Serial.print(imuConfig.gyro.powerDown ? "True" : "False");
  Serial.print(", Sleep Enabled: "); Serial.print(imuConfig.gyro.sleepEnabled ? "True" : "False");
  Serial.print(", Low Power Enabled: "); Serial.println(imuConfig.gyro.lowPowerEnabled ? "True" : "False");
  Serial.print("Full Scale (0 = 245, 1 = 500, 2 = 2000 DPS): "); Serial.println(imuConfig.gyro.scale);
  Serial.print("ODR (0 = OFF, 1 = 14.9, 2 = 59.5, 3 = 119, 4 = 238, 5 = 476, 6 = 952 Hz): "); 
  Serial.println(imuConfig.gyro.sampleRate);
  Serial.print("Bandwidth (0 = LOW, 1 = MID, 2 = HIGH, 3 = MAX): "); Serial.println(imuConfig.gyro.bandwidth);
  Serial.print("Orientation: (0 = XYZ): "); Serial.print(imuConfig.gyro.orientation);
  Serial.print(", Sign (0 = NONE): "); Serial.println(imuConfig.gyro.reverseSign);
  Serial.print("Raw Bias Offset, X: "); Serial.print(imuConfig.gyro.bias.x);
  Serial.print(", Y: "); Serial.print(imuConfig.gyro.bias.y);
  Serial.print(", Z: "); Serial.println(imuConfig.gyro.bias.z);

  Serial.println("\n  ========  ACCEL Configuration    ========");
  Serial.print("Power Down (True in NORMAL mode): "); Serial.println(imuConfig.accel.powerDown ? "True" : "False");
  Serial.print("Enable Auto Bandwidth: "); Serial.println(imuConfig.accel.autoBandwidthEnable ? "True" : "False");
  Serial.print("Scale (0 = 2, 1 = 16, 2 = 4, 3 = 8 G's): "); Serial.println(imuConfig.accel.scale);
  Serial.print("ODR (0 = OFF, 1 = 10, 2 = 50, 3 = 119, 4 = 238, 5 = 476, 6 = 952, 7 = Gyro ODR): "); 
  Serial.println(imuConfig.accel.sampleRate);
  Serial.print("Bandwidth (0 = 408, 1 = 211, 2 = 105, 3 = 50 Hz): "); 
  Serial.println(imuConfig.accel.bandwidth);
  Serial.print("Raw Bias Offset, X: "); Serial.print(imuConfig.accel.bias.x);
  Serial.print(", Y: "); Serial.print(imuConfig.accel.bias.y);
  Serial.print(", Z: "); Serial.println(imuConfig.accel.bias.z);

  Serial.println("\n  ========  MAG Configuration    ========");
  Serial.print("Fast ODR Enabled: "); Serial.print(imuConfig.mag.fastODR ? "True" : "False");
  Serial.print(", Temp Compensation Enabled: "); Serial.println(imuConfig.mag.temperatureCompensated ? "True" : "False");
  Serial.print("Operating Mode (0 = LOW, 1 = MED, 2 = HIGH, 3 = ULTRA): "); 
  Serial.println(imuConfig.mag.opMode);
  Serial.print("Sample Mode (0 = CONT, 1 = SINGLE, 2 = IDLE): "); 
  Serial.println(imuConfig.mag.sampleMode);
  Serial.print("Scale (0 = 4, 1 = 8, 2 = 12, 3 = 16 gauss): "); 
  Serial.println(imuConfig.mag.scale);
  Serial.println("ODR (0 = 0.625, 1 = 1.25, 2 = 2.5, 3 = 5, 4 = 10, 5 = 20, 6 = 40, 7 = 80 Hz..."); 
  Serial.print("...FAST ODR 8 = 155, 9 = 300, 10 = 560, 11 = 1000 Hz): ");
  Serial.println(imuConfig.mag.sampleRate);
  Serial.print("Raw Bias Offset, X: "); Serial.print(imuConfig.mag.bias.x);
  Serial.print(", Y: "); Serial.print(imuConfig.mag.bias.y);
  Serial.print(", Z: "); Serial.println(imuConfig.mag.bias.z);

  Serial.println("\n  ========  TEMPERATURE Configuration    ========");
  Serial.print("Temp Offset: "); Serial.print(imuConfig.temp.offset); Serial.println("°C\n");
  Serial.println("-----------------------------------------------------------------------------\n");
}

void readAnswer(char msg[], int& param)
{ char ch=0;
  byte count=0;
  const byte NofChars = 8;
  char ans[NofChars];
//  float val;
  while (Serial.available()){Serial.read();}  //empty read buffer
  Serial.print(msg); 
  Serial.print(param); 
  Serial.print(F(" Enter new value ")); 
  while (byte(ch)!=10 && byte(ch)!=13 && count<(NofChars-1)   )
  {  if (Serial.available())
     {  ch= Serial.read();
        ans[count]=ch;
        count++;
     }
  }      
  ans[count]=0;
  Serial.println(ans);
  if (count>1) param= atoi(ans);
  while (Serial.available()){Serial.read();}
     Serial.println("\n\n\n\n\n\n\n"); 
}
