#include "sensors.h"
#include "navigation.h"

// coord_t route[MAX_WAYPOINTS]; 

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
  //delay(1000);
  //sIMU();
  delay(1000);
  nShort();
  delay(1000);
  nServos();
}


