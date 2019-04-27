#include "sensors.h"
#include "navigation.h"

/*-----------------------------------------------------------------
 CUSail
`Cornell Autonomous Sailboat Team

================
VERSION 17
================
Authors: Kurt Huebner, Troy Smith, Beth Mieczkowski

 ShortTermNavigation
  The primary file that schedules tasks for SailVane:

  0. Initialization
  1. Obtain Rotary Sensor data
  2. Obtain GPS data
  3. Obtain IMU data
  4. Obtain appropriate sail angle and tailvane angles
  5. Set servos to obtained sail and tailvane angles
  6. Go to 1.

 Code has been tested and run on an Arduino Due
--------------------------------------------------------------------*/

// Code within setup() runs once on receiving power
void setup() {
  initSensors();
  initializer();
}

// Code within loop() runs after setup and runs constantly (in order) while the microcontroller is powered
void loop() {
  sRSensor(); //Gather wind directio w.r.t North
  sGPS(); //Gather global coordinates
  sIMU(); //Gather boat direction w.r.t North, Roll and Pitch
  nav(); //Run Short Term Navigation Algorithm to obtain servo commands
}
