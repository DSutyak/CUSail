#include "sensors.h"
#include "navigation.h"

/* Wind has been set to 290 degrees from North and Boat direction has been set to 40 degrees from North. 
 IMU is not being run*/

void setup() {
  initSensors();
  initNavigation();
  initServos();
  setWaypoints();
}

void loop() {
  sRSensor();
  sGPS();
  //sIMU();
  nShort();
  nServos();
}


