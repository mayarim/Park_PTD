# Park_PTD
Cueing device for Parkinson's patients in physical therapy for gait training.
This repository holds the Arduino code for this project. This code was written and tested on the Arduino Nano 33 BLE Rev1 and Rev2.

The main functionality of the code in this repository is to utilize the Arduino Nano 33 BLE's built-in 9-axis IMU to detect steps when worn on or near the ankle and calculate the length of each step taken. If that step is below a given threshold haptic feedback is provided via the vibration disc motor. This provides a cue to patients wearing the device to lengthen their steps.

## Hardware
Integration of the physical components is shown in the following figure:

<img src="https://github.com/mayarim/Park_PTD/assets/75183224/f88b7426-1d35-448c-9af0-bba432eb4d0e" width="300">

### (1) Ankle Strap
This can be made using a plastic ring, a length of velcro, and a stapler. First, loop the soft side of the velcro through the ring and stapling to secure it. On the other end secure a smaller length of the hard side of the velcro. The total length should work out to wrap around the ankle without too much overlap.
### (2) Arduino Nano 33 BLE rev1/rev2
The Arduino Nano 33 BLE was chosen for its small form factor while having a 9-axis IMU and BLE module with enough processing power to run any algorithm desired. The Arduino Nano is mounted on a 3D-printed mount (casing_3Dmodels/nano_board_holder_v3.SLDPRT). Screw holes were added to the model, but hot glue works for mounting as well.
### (3) DRV2605L Motor Controller
- DRV2605L Haptic Motor Driver https://www.adafruit.com/product/2305

This motor controller drives the Mini Vibration Motor and provides many preset options in the software to allow customization of the haptic feedback. This is mounted on a 3D-printed mount (casing_3Dmodels/haptic_driver_holdre.SLDPRT). It is fastened using hot glue to the mount. 
The motor driver's VIN, GND, SDA, and SCL pins are wired to the 3.3V, GND, A4, and A5 pins on the Arduino Nano.
### (4) Mini Vibration Disc Motor
- Mini Vibration Disc Motor https://www.adafruit.com/product/1201

Any mini-vibration motor should work as long as it can be driven by the 3.3V output. A longer length of wire between the motor controller and the motor allows the user to place the haptic feedback wherever works best for them. Hot glue is used as strain relief for soldering points.
### (5) 9 Volt Battery. 
A 9-volt battery with a battery clip is used to power the device. A casing was made for the battery (casing_3Dmodels/battery_box_1.SLDPRT) but was further reinforced with duct tape to help reduce movement.
### (6) Switch
This is not shown in the image above, but a toggle switch can be added between the battery and the battery so the device can be easily turned on and off. Hot gluing this to the side of the battery casing ensures minimal movement, and adding hot glue to the soldering point on the switch provides strain relief.

_All CAD files for 3D-printed casing designs are in the **casing_3Dmodels** folder._

## Using the System
Assuming that the correct min_length (or min_len_r2) is uploaded to the device, there is an initial startup sequence for the device. Within this 10-second window, the desired step length can be sent to the device, this period is indicated by an orange light on the Arduino board itself. Once the orange light is off, the device calculates the step length algorithm continuously. If the orange light does not turn off after 10 seconds, the startup sequence likely failed and the system should be powered down and then powered on to try again. When turning on the device a green light should also turn on, this is the power indicator, if there is no green light the system is not on. A reason for the system not being on despite the battery being plugged in is a lack of power in the battery or that the board is broken.

**Note**: The version of the code with the Arduino Nano 33 BLE rev1 should be turned on in an orientation such that the pins on the board are sticking down at the ground. Holding the device up against the underside of a table or flat surface works well. This is for the best performance, not doing so may result in erroneous measurements. This is unnecessary for the rev2.

## Uploading Code
The Arduino Nano 33 BLE can be programmed using a micro-USB connected to a computer running the Arduino IDE. When uploading the sketches that have BLE functionality, note that there are lines that should be commented/uncommented specifically for the left and right devices. These lines of code help identify the devices when attempting to connect to the corresponding iOS app.

## Arduino Sketches
**Note**: The *_r2* version of these Arduino sketches is the version of the sketch for the Arduino Nano 33 BLE Rev2. The sketches without *_r2* at the end are for Rev1. This difference is important as the Rev2 contains a different IMU than the Rev1 and the Rev1 sketches use the Reefwing_LSM9DS1 library to configure the IMU and access the data. The Rev2 sketches use the standard Arduino library Arduino_BMI270_BMM150 to retrieve the IMU values.
### calculate_length / calc_len_r2
This contains code that prints out various values to the connected serial monitor. Its primary use is as a debugger to understand what values are being calculated and to visualize the data being seen. These values include the raw accelerometer, raw gyroscope, normalized acceleration (without gravity), and step calculations. These can be switched between by typing various letters into the serial monitor input such as 'g', 'a', 'n', etc. The full list can be seen in the code.

### send_length / send_len_r2
This code is functionally the same as calculate_length except that code for BLE connection is implemented for testing.

### min_length / min_len_r2
Contains the minimized version of send_length without any reliance on the serial monitor and is the version that should be used when attempting to use the device when relying on battery power. We've found that leaving any statements involving serial output causes the device to not work when not connected to a serial output such as the computer.

### ahrs_visualizer
This is *NOT* an Arduino sketch. It is a Processing (https://processing.org/) sketch and is used to visualize the output of the AHRS to see if the values correctly match the orientation of the device in real life.


## Files
The files in each sketch other than the main file are identical and are helpers to the main file.
### Main File: calculate_length.ino / send_len.ino / min_len.ino
These files are the main sketch files and contain the overall setup and loop logic. The primary calculations for the step length detection and measurement are all implemented in these respective files.
### Filter.h + Filter.cpp
These files define the interface and implement two classes used in the main files: Filter and ZVU.
#### Filter
This class implements a low-pass filter for iterative data based on initialized parameters.
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
