# Park_PTD
Cueing device for Parkinson's patients in physical therapy for gait training.
This repository holds the Arduino code for this project. This code was written and tested on the Arduino Nano 33 BLE Rev1 and Rev2, available at https://store-usa.arduino.cc/products/nano-33-ble-sense-rev2. 

Other hardware components include:
- DRV2605L haptic motor driver https://www.adafruit.com/product/2305
- Mini Vibration Disc Motor https://www.adafruit.com/product/1201

The main functionality of the code in this repository is to utilize the Arduino Nano 33 BLE's built-in 9-axis IMU to detect steps when worn on or near the ankle and calculate the length of each step taken. If that step is below a given threshold haptic feedback is provided via the vibration disc motor. This provides a cue to patients wearing the device to lengthen their steps.

Integration of the physical components is shown in the following figure:

<img src="https://github.com/mayarim/Park_PTD/assets/75183224/f88b7426-1d35-448c-9af0-bba432eb4d0e" width="300">

Components: (1) Ankle Strap, (2) Arduino Nano 33 BLE, (3) DRV2605L Motor Controller, (4) Mini Vibration Disc Motor, (5) 9 Volt Battery. 

CAD files for 3D-printed casing designs are in the **casing_3Dmodels** folder.

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

## Files
The files in each sketch other than the main file are identical and are helpers to the main file.
### Main File: calculate_length.ino / send_len.ino / min_len.ino
These files are the main sketch files and contain the overall setup and loop logic. The primary calculations for the step length detection and measurement are all implemented in these respective files.
### Filter.h + Filter.cpp
These files define the interface and implement two classes used in the main files: Filter and ZVU.
#### Filter
This is a class that implements a low-pass filter for data based on inputted parameters.
#### ZVU
This outputs a 0 or 1 based on whether there has been significant acceleration activity recently. This was based on the Zero Velocity Update algorithm from https://github.com/Wojtek120/IMU-velocity-and-displacement-measurements/tree/master.
### Integral.h + Integral.cpp
These files define and implement the Integral and DeltaTime classes.
#### Integral
Performs discrete integration using trapezoidal summation. Every input via the step() function is an integration step.
#### DeltaTime
This is a helper class for Integral which helps to calculate the time in seconds between IMU updates. This also converts the measurements from microseconds to seconds.
### Utility.ino
This contains miscellaneous methods that abstract away some repeatedly used code segments. 

## Libraries Used
- https://www.arduino.cc/reference/en/libraries/reefwingahrs/
- https://www.arduino.cc/reference/en/libraries/reefwing_imutypes/
- https://www.arduinolibraries.info/libraries/reefwing-lsm9-ds1
- https://www.arduino.cc/reference/en/libraries/arduinoble/
- https://www.arduino.cc/reference/en/libraries/arduino_bmi270_bmm150/
- https://www.arduino.cc/reference/en/libraries/adafruit-drv2605-library/
- https://www.arduino.cc/reference/en/libraries/vector-datatype/
