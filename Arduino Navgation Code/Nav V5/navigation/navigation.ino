#include "sensors.h"
#include "navigation.h"

/* Wind has been set to 315 degrees from North and Boat direction has been set to 245 degrees from North. 
 IMU is not being run*/

void setup() {
  initSensors();
  initNavigation();
  initServos();
  setWaypoints();
}

void loop() {
  sRSensor();
  delay(1000);
  sGPS();
  delay(1000);
  //sIMU();
  //delay(1000);
  nShort();
  delay(1000);
  nServos();
  delay(1000);
}


