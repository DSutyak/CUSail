#include "sensors.h"
#include "navigation.h"

// coord_t route[MAX_WAYPOINTS]; 

// NOTE: Serial1.print() for XBees
// NOTE: (later) possibly LED for watchdog (Arjan is very anti-watchdog) 


// LED usage:
//    - green: power
//    - blue: waypoint reached
//    - red1: in sensor gathering code
//    - red2: 
//    - yellow: 

void setup() {
  initSensors();
  initNavigation();
  initServos();
  // setWaypoints(route);
  setWaypoints();
}

void loop() {
  digitalWrite(redLED1,LOW);
  delay(1000);
  digitalWrite(redLED1,HIGH);
  sRSensor();
  //delay(1000);
  sGPS();
  //delay(1000);
  sIMU();
  //delay(1000);
  nShort();
  //delay(1000);
  nServos();
}


