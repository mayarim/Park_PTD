/*
 This sketch shows to rotate the heading (yaw angle) of the estimated orientation.
 */

#include <Arduino_LSM9DS1.h>
#include <imuFilter.h>

imuFilter fusion;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    // stop here if you can't access the IMU:
    while (true);
  }

  IMU.setGyroFS(2);
  IMU.setGyroODR(5); // 476Hz
  IMU.setGyroOffset (3.615631, -0.771332, 1.388794);
  IMU.setGyroSlope (1.193819, 1.141039, 1.154469);
  IMU.gyroUnit= DEGREEPERSECOND;

  IMU.setAccelFS(3);
  IMU.setAccelODR(5); // 476Hz
  IMU.setAccelOffset(-0.016153, -0.025595, -0.027484);
  IMU.setAccelSlope (1.003992, 0.995510, 1.002365);
  IMU.accelUnit=  GRAVITY;
  
  float xAcc, yAcc, zAcc;
//  float xGyro, yGyro, zGyro;
  // Initialize filter: 
//  fusion.setup( imu.ax(), imu.ay(), imu.az() );   
  while(true){
    if (IMU.accelAvailable()) {
      // read accelerometer &and gyrometer:
      IMU.readAccel(xAcc, yAcc, zAcc);
//      IMU.readGyro(xGyro, yGyro, zGyro);
      break;
    }
  }
    // update the filter, which computes orientation:
    fusion.setup(xAcc, yAcc, zAcc);  
                  
  // Rotate heading:
  float angle = 45 * DEG_TO_RAD;                // angle in radians to rotate heading about z-axis
  fusion.rotateHeading( angle, LARGE_ANGLE );   // Can choose LARGE_ANGLE or SMALL_ANGLE approximation
}

void loop() {  
//  // Update filter:
//  fusion.update( imu.gx(), imu.gy(), imu.gz(), imu.ax(), imu.ay(), imu.az() );    
//
//  // Display angles:
//  Serial.print( fusion.pitch() );
//  Serial.print( " " );
//  Serial.print( fusion.yaw() );
//  Serial.print( " " );
//  Serial.print( fusion.roll() );
//  Serial.println();

  // values for acceleration and rotation:
  float xAcc, yAcc, zAcc;
  float xGyro, yGyro, zGyro;
//  float xMag, yMag, zMag;

  // values for orientation:
//  float roll, pitch, heading;
  // check if the IMU is ready to read:
  if (IMU.accelAvailable() &&
      IMU.gyroAvailable()) {
    // read accelerometer &and gyrometer:
    IMU.readAccel(xAcc, yAcc, zAcc);
//    IMU.readRawAccel(xAcc, yAcc, zAcc);
    IMU.readGyro(xGyro, yGyro, zGyro);
//    IMU.readRawGyro(xGyro, yGyro, zGyro);
//    IMU.readMagnet(xMag, yMag, zMag);
//    IMU.readRawMagnet(xMag, yMag, zMag);

    // update the filter, which computes orientation:
    fusion.update(xGyro, yGyro, zGyro, xAcc, yAcc, zAcc);

    // print the heading, pitch and roll
//    roll = filter.getRoll();
//    pitch = filter.getPitch();
//    heading = filter.getYaw();
    Serial.print("Orientation: ");
    Serial.print(fusion.yaw());
    Serial.print(" ");
    Serial.print(fusion.pitch());
    Serial.print(" ");
    Serial.println(fusion.roll());
  }
}
