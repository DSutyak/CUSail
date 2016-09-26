#include "sensors.h"
#include "navigation.h"

/*-----------------------------------------------------------------
 CU Sail
`Cornell University Autonomous Sailboat Team

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

 Last Updated: 9/25/2016 by Arjan Singh
 
 Authors: Alex Pomerenk, Alec Dean, Arjan Singh, Stephanie Hogan
 
 Past Contributors: Eric T. J. Jung, Brian Gross, Varun Shanbhag, David Brown
--------------------------------------------------------------------*/

void setup() {
  initSensors();
  initNavigation();
  initServos();
  setWaypoints();
}

void loop() {
  delay(1000);
  sRSensor();
  delay(1000);
  sGPS();
  delay(1000);
  sIMU();
  delay(1000);
  nShort();
  delay(1000);
  nServos();
}
