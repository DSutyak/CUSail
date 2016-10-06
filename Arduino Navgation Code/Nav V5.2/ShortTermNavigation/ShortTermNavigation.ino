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

 Last Updated: 10/6/2016 by Arjan Singh
 
 Authors: Alex Pomerenk, Alec Dean, Arjan Singh, Stephanie Hogan
 
 Past Contributors: Eric T. J. Jung, Brian Gross, Varun Shanbhag, David Brown
--------------------------------------------------------------------*/

// Code within setup() runs once on receiving power
void setup() {
  initSensors(); // Run setup code for all sensors
  initNavigation(); // Set the number of waypoints and the current waypoint # to 0
  initServos(); // attach the sail and tailvane servos
  setWaypoints(); // create the course by means of a waypoint array
}

// Code within loop() runs after setup and runs constantly (in order) while the mircrocontroller is powered
void loop() {
  sRSensor(); //Gather wind directio w.r.t North
  sGPS(); //Gather global coordinates
  sIMU(); //Gather boat direction w.r.t North, Roll and Pitch
  nShort(); //Run Short Term Navigation Algorithm to obtain servo commands
  nServos(); //Send acquired servo commands
}
