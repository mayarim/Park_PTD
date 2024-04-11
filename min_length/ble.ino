BLEDevice c;
void bleFetch(){
//    BLEDevice c;
    c = BLE.central();
//  // if central not connected
  if(!c || !c.connected()){
//    Serial.println("bleFetch: No device available!!!");
    return;
  }
//  // If value available
  if(writeCharacteristic.written()){
    step_target = writeCharacteristic.value();
//    Serial.print("New step length target: ");
//    Serial.println(step_target);
  }
}

void bleSend(String data){
//  BLEDevice c;
  c = BLE.central();
  // if not connected
  if(!c || !c.connected()){
    //Serial.println("BleSend: Data not sent!!!");
    return;
  }
  dataCharacteristic.writeValue(data);
//  Serial.print("Data sent: ");
//  Serial.println(data);
}
