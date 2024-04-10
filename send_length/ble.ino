void bleFetch(BLEDevice central){
  // if central not connected
  if(!central || !central.connected()){
    Serial.println("bleFetch: No device available!!!");
    return;
  }
  // If value available
  if(writeCharacteristic.written()){
    step_target = writeCharacteristic.value();
    Serial.print("New step length target: ");
    Serial.println(step_target);
  }
}

void bleSend(BLEDevice central, String data){
  // if not connected
  if(!central || !central.connected()){
    Serial.println("BleSend: Data not sent!!!");
    return;
  }
  dataCharacteristic.writeValue(data);
  Serial.print("Data sent: ");
  Serial.println(data);
}
