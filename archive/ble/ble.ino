#include <ArduinoBLE.h>

// BLE Data Service
BLEService dataService("180A");

// BLE Data Characteristic - custom 128-bit UUID, read and notify
BLEStringCharacteristic dataCharacteristic("0000AAAA-0000-1000-8000-00805F9B34FB", BLERead | BLENotify, 128); // 20 is the maximum length of the string

// New characteristic to be written to by external devices
BLEByteCharacteristic writeCharacteristic("0000BBBB-0000-1000-8000-00805F9B34FB", BLEWrite);

unsigned long previousMillis = 0;  // will store last time data was updated
const long interval = 1000;  // interval at which to update data (milliseconds)

int timeValue = 0; // Time value in seconds
int stepLength = 0; // Step length value

void setup() {
  // set LED's pin to output mode
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // turn off the built-in LED

  // begin initialization
  if (!BLE.begin()) {
    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("NanoParkinsons");
  BLE.setAdvertisedService(dataService);
  BLE.setAdvertisingInterval(100);

  // add the characteristics to the service
  dataService.addCharacteristic(dataCharacteristic);
  dataService.addCharacteristic(writeCharacteristic);

  // add service
  BLE.addService(dataService);

  // start advertising
  BLE.advertise();
}

void blinkWhite() {
  // Blink white (built-in LED on) for 1 second
  digitalWrite(LED_BUILTIN, HIGH); // turn on the built-in LED
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW); // turn off the built-in LED
}

void loop() {
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    // while the central is still connected to peripheral:
    while (central.connected()) {
      // Check if there is a value written to the writeCharacteristic
      if (writeCharacteristic.written()) {
        byte newValue = writeCharacteristic.value();
        blinkWhite(); // Blink white when a message is received
      }

      unsigned long currentMillis = millis();

      // Check if it's time to update the characteristic value
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        // Increment time value and generate a random step length
        timeValue++;
        stepLength = random(0, 101); // Random step length from 0 to 100

        // Create the data string in the format "time,step length"
        String dataString = String(timeValue) + "," + String(stepLength);

        // Write the data string to the characteristic
        dataCharacteristic.writeValue(dataString);

        // Print the data string for debugging
      //  Serial.println(dataString);
      }
    }

    digitalWrite(LED_BUILTIN, LOW); // turn off the built-in LED when the central disconnects
  }
}
