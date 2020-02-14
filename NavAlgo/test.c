/**
 * Sensor test functions - all return strings to be displayed on tft board
 */

#include <stdio.h>
#include "sensors.h"

/**
 * Wind direction and wind speed
 */
char* testAnemometer() {
  static char buffer[60];
  readAnemometer();
  sprintf(buffer, "wind dir: %3d, wind speed: %2.2f", sensorData->wind_dir, sensorData->wind_speed);
  return buffer;
}

/**
 * Distance to closest object
 */
char* testLidar() {
  static char buffer[60];
  float lidarDistance = readLIDAR();
  sprintf(buffer,"distance: %4.2f", lidarDistance);
  return buffer;
}

///**
// * Latitude and longitude
// */
//char* testGPS() {
//  static char buffer[60];
//  readGPS();
//  sprintf(buffer,"lat: %d, long: %d", sensorData->lat, sensorData->longi);
//  return buffer;
//}

/**
 * Pitch, roll, and yaw (boat heading)
 */
char* testIMU() {
  static char buffer[60];
  readIMU();
  sprintf(buffer,"yaw: %d, pitch: %d, roll: %d", sensorData->boat_direction, sensorData->pitch, sensorData->roll);
  return buffer;
}
