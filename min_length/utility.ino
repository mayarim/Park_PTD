vec3_t getAccel(SensorData data){
  return {data.ax, data.ay, data.az};
}

vec3_t getGyro(SensorData data){
  return {data.gx, data.gy, data.gz};
}

String buildDataString(uint32_t ts, float slen){
  return String(ts * MICRO_TO_SEC) + "," + String(slen * M_TO_CM);
}

void printVector(char msg[], vec3_t vec){
  //Serial.print(msg);
 // Serial.print(vec.x);
 // Serial.print(" ");
 // Serial.print(vec.y);
 // Serial.print(" ");
 // Serial.print(vec.z);
 // Serial.print(" ");
 // Serial.println(vec.mag());
}
//
void printVector(char msg[], vec3_t vec, float a){
 // Serial.print(msg);
  //Serial.print(vec.x);
 // Serial.print(" ");
 // Serial.print(vec.y);
 // Serial.print(" ");
 // Serial.print(vec.z);
 // Serial.print(" ");
  //Serial.print(vec.mag());
  //Serial.print(" ");
  //Serial.println(a);
}
//
//void printVector(char msg[], float a, float b, float c){
//  Serial.print(msg);
//  Serial.print(a);
//  Serial.print(" ");
//  Serial.print(b);
//  Serial.print(" ");
//  Serial.println(c);
////  Serial.print(" ");
////  Serial.println(sqrt(a*a + b*b + c*c));
//}
//
char readChar()
{  char ch;
   ch= toupper(Serial.read());   
   delay(10);
   while (Serial.available()){Serial.read();delay(1);} // empty readbuffer 
   return ch;
}

//void printConfig() {
//  Serial.println("\n  ========  IMU Configuration    ========");
//  Serial.print("Gyro/Accel FIFO Enabled: "); Serial.println(imuConfig.gyroAccelFIFOEnabled ? "True" : "False");
//  Serial.print("FIFO Mode (0 = BYPASS, 1 = FIFO_THS, 3 = CONT_TO_FIFO, 4 = BYPASS_TO_CONT, 6 = FIFO_CONT): "); 
//  Serial.println(imuConfig.fifoMode);
//  Serial.print("FIFO Threshold: "); Serial.println(imuConfig.fifoThreshold);
//  Serial.print("Gyro/Accel Operating Mode (0 = OFF, 1 = ACCEL, 2 = NORMAL, 3 = LOW_PWR): "); 
//  Serial.println(imuConfig.gyroAccelOpMode);
//
//  Serial.println("\n  ========  GYRO Configuration    ========");
//  Serial.print("Power Down: "); Serial.print(imuConfig.gyro.powerDown ? "True" : "False");
//  Serial.print(", Sleep Enabled: "); Serial.print(imuConfig.gyro.sleepEnabled ? "True" : "False");
//  Serial.print(", Low Power Enabled: "); Serial.println(imuConfig.gyro.lowPowerEnabled ? "True" : "False");
//  Serial.print("Full Scale (0 = 245, 1 = 500, 2 = 2000 DPS): "); Serial.println(imuConfig.gyro.scale);
//  Serial.print("ODR (0 = OFF, 1 = 14.9, 2 = 59.5, 3 = 119, 4 = 238, 5 = 476, 6 = 952 Hz): "); 
//  Serial.println(imuConfig.gyro.sampleRate);
//  Serial.print("Bandwidth (0 = LOW, 1 = MID, 2 = HIGH, 3 = MAX): "); Serial.println(imuConfig.gyro.bandwidth);
//  Serial.print("Orientation: (0 = XYZ): "); Serial.print(imuConfig.gyro.orientation);
//  Serial.print(", Sign (0 = NONE): "); Serial.println(imuConfig.gyro.reverseSign);
//  Serial.print("Raw Bias Offset, X: "); Serial.print(imuConfig.gyro.bias.x);
//  Serial.print(", Y: "); Serial.print(imuConfig.gyro.bias.y);
//  Serial.print(", Z: "); Serial.println(imuConfig.gyro.bias.z);
//
//  Serial.println("\n  ========  ACCEL Configuration    ========");
//  Serial.print("Power Down (True in NORMAL mode): "); Serial.println(imuConfig.accel.powerDown ? "True" : "False");
//  Serial.print("Enable Auto Bandwidth: "); Serial.println(imuConfig.accel.autoBandwidthEnable ? "True" : "False");
//  Serial.print("Scale (0 = 2, 1 = 16, 2 = 4, 3 = 8 G's): "); Serial.println(imuConfig.accel.scale);
//  Serial.print("ODR (0 = OFF, 1 = 10, 2 = 50, 3 = 119, 4 = 238, 5 = 476, 6 = 952, 7 = Gyro ODR): "); 
//  Serial.println(imuConfig.accel.sampleRate);
//  Serial.print("Bandwidth (0 = 408, 1 = 211, 2 = 105, 3 = 50 Hz): "); 
//  Serial.println(imuConfig.accel.bandwidth);
//  Serial.print("Raw Bias Offset, X: "); Serial.print(imuConfig.accel.bias.x);
//  Serial.print(", Y: "); Serial.print(imuConfig.accel.bias.y);
//  Serial.print(", Z: "); Serial.println(imuConfig.accel.bias.z);
//
//  Serial.println("\n  ========  MAG Configuration    ========");
//  Serial.print("Fast ODR Enabled: "); Serial.print(imuConfig.mag.fastODR ? "True" : "False");
//  Serial.print(", Temp Compensation Enabled: "); Serial.println(imuConfig.mag.temperatureCompensated ? "True" : "False");
//  Serial.print("Operating Mode (0 = LOW, 1 = MED, 2 = HIGH, 3 = ULTRA): "); 
//  Serial.println(imuConfig.mag.opMode);
//  Serial.print("Sample Mode (0 = CONT, 1 = SINGLE, 2 = IDLE): "); 
//  Serial.println(imuConfig.mag.sampleMode);
//  Serial.print("Scale (0 = 4, 1 = 8, 2 = 12, 3 = 16 gauss): "); 
//  Serial.println(imuConfig.mag.scale);
//  Serial.println("ODR (0 = 0.625, 1 = 1.25, 2 = 2.5, 3 = 5, 4 = 10, 5 = 20, 6 = 40, 7 = 80 Hz..."); 
//  Serial.print("...FAST ODR 8 = 155, 9 = 300, 10 = 560, 11 = 1000 Hz): ");
//  Serial.println(imuConfig.mag.sampleRate);
//  Serial.print("Raw Bias Offset, X: "); Serial.print(imuConfig.mag.bias.x);
//  Serial.print(", Y: "); Serial.print(imuConfig.mag.bias.y);
//  Serial.print(", Z: "); Serial.println(imuConfig.mag.bias.z);
//
//  Serial.println("\n  ========  TEMPERATURE Configuration    ========");
//  Serial.print("Temp Offset: "); Serial.print(imuConfig.temp.offset); Serial.println("°C\n");
//  Serial.println("-----------------------------------------------------------------------------\n");
//}
