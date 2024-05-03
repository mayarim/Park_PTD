# Park_PTD
Cueing device for Parkinson's patients in physical therapy for gait training.
This repository holds the Arduino code for this project. This code was written and tested on the Arduino Nano 33 BLE Rev1 and Rev2. Other hardware components include:
- DRV2605L haptic motor driver
- Mini Vibration Disc Motor

The main functionality of the code in this repository is to utilize the Arduino Nano 33 BLE's built-in 9-axis IMU to detect steps when worn on or near the ankle and calculate the length of each step taken. If that step is below a given threshold haptic feedback is provided via the vibration disc motor. This provides a cue to patients wearing the device to lengthen their steps.

## Arduino Sketches
### calculate_length / calc_len_r2
This contains code that prints out various values to the connected serial monitor. Its primary use is as a debugger to understand what values are being calculated and to visualize the data being seen. These values include the raw accelerometer, raw gyroscope, normalized acceleration (without gravity), and step calculations. These can be switched between by typing various letters into the serial monitor input such as 'g', 'a', 'n', etc. The full list can be seen in the code.

### send_length / send_len_r2
This code is functionally the same as calculate_length except that code for BLE connection is implemented for testing.

### min_length / min_len_r2
Contains the minimized version of send_length without any reliance on the serial monitor and is the version that should be used when attempting to use the device when relying on battery power. We've found that leaving any statements involving serial output causes the device to not work when not connected to a serial output such as the computer.

### ahrs_visualizer
This is *NOT* an Arduino sketch. It is a Processing (https://processing.org/) sketch and is used to visualize the output of the AHRS to see if the values correctly match the orientation of the device in real life.

### Note
The *_r2* version of these Arduino sketches is the version of the sketch for the Arduino Nano 33 BLE Rev2. The sketches without *_r2* at the end are for Rev1. This difference is important as the Rev2 contains a different IMU than the Rev1 and the Rev1 sketches use the Reefwing_LSM9DS1 library to configure the IMU and access the data. The Rev2 sketches use the standard Arduino library Arduino_BMI270_BMM150 for access to the IMU values.

## Libraries Used
- https://www.arduino.cc/reference/en/libraries/reefwingahrs/
- https://www.arduino.cc/reference/en/libraries/reefwing_imutypes/
- https://www.arduinolibraries.info/libraries/reefwing-lsm9-ds1
- https://www.arduino.cc/reference/en/libraries/arduinoble/
- https://www.arduino.cc/reference/en/libraries/arduino_bmi270_bmm150/
- https://www.arduino.cc/reference/en/libraries/adafruit-drv2605-library/
- https://www.arduino.cc/reference/en/libraries/vector-datatype/
