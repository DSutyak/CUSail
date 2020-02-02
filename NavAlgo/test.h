/**
 * Sensor test functions
*/

#ifndef TEST_H
#define TEST_H

/**
 * Wind direction and wind speed
 */
char* testAnemometer();

/**
 * Distance to closest object
 */
char* testLidar();

/**
 * Latitude and longitude
 */
char* testGPS();

/**
 * Pitch, roll, and yaw (boat heading)
 */
char* testIMU();

#endif
