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

void toMetersTest(void) {
    toMeters(42.444299, -76.482556, 42.444304, -76.482891); // 26.93
  // http://www.movable-type.co.uk/scripts/latlong.html 
  //   0.02749 km = 27.49 m

  toMeters(42.444299, -76.482556, 42.444320, -76.483670); // 91.44
  // 0.09144 km = 91.44 m

  // 42.444298, -76.482559 to 0,0   => actual 8903 km  
  // 8903114.00 m = 8903.114 km = very close 

  // function works fine; maybe problem with storage in sensorData
  // OR input wrong lat/long coordinates 


}


// 42.444346, -76.483264  -- wp1 actual, down stairs, kimball entrance to sidewalk across quad
// bottomStairs = {42.444240, -76.483258},

void loop() {
  digitalWrite(redLED1,LOW);
  delay(4000);
  digitalWrite(redLED1,HIGH);
  digitalWrite(blueLED,LOW);

  //toMetersTest();

  
  sRSensor();
  //delay(1000);
  sGPS();
  //delay(1000);
  sIMU();
  //delay(1000);
  nShort();
//  Serial.print("** Current location: "); Serial.print(sensorData.lati,6); Serial.print(", "); 
//  Serial.println(sensorData.longi,6);
//  Serial.print("** Next waypoint   : "); Serial.print(wayPoints[wpNum].latitude,6); Serial.print(", "); 
//  Serial.println(wayPoints[wpNum].longitude,6);
  //delay(1000);
  nServos();
}


