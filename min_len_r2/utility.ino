vec3_t getAccel(SensorData data){
  return {data.ax, data.ay, data.az};
}

vec3_t getGyro(SensorData data){
  return {data.gx, data.gy, data.gz};
}

String buildDataString(uint32_t ts, float slen){
  return String(ts * MICRO_TO_SEC) + "," + String(slen * M_TO_CM);
}

//void printVector(char msg[], vec3_t vec){
//  //Serial.print(msg);
// // Serial.print(vec.x);
// // Serial.print(" ");
// // Serial.print(vec.y);
// // Serial.print(" ");
// // Serial.print(vec.z);
// // Serial.print(" ");
// // Serial.println(vec.mag());
//}
//
//void printVector(char msg[], vec3_t vec, float a){
// // Serial.print(msg);
//  //Serial.print(vec.x);
// // Serial.print(" ");
// // Serial.print(vec.y);
// // Serial.print(" ");
// // Serial.print(vec.z);
// // Serial.print(" ");
//  //Serial.print(vec.mag());
//  //Serial.print(" ");
//  //Serial.println(a);
//}
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
