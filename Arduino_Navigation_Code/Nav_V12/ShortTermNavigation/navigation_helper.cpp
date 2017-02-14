#include <Arduino.h>
#include <math.h>
#include "navigation.h"

/*------------------------------------------*/
/*----------Tailvane-angle setters----------*/
/*------------------------------------------*/

// orientation is 1 for right and 0 for left
// right is negative offset, left is positive
// never go upright when facing left
// facing right, angle is above in the sector: w-offset
//   w-(|w+opttop - boatdir|)
float upRight(float b, float w) {
  float offset = fabsf(w+optPolarTop-b);
  tailAngle=w-offset;
  // sailAngle=tailAngle+angleofattack;
  return tailAngle;
}

float rightTarget(float b, float w){
  // sailAngle=sensorData.windDir+angleofattack;
  tailAngle=w;
  return tailAngle;
}

float leftTarget(float b, float w){
  // sailAngle=sensorData.windDir-angleofattack;
  tailAngle=w;
  return tailAngle;
}

// facing left, angle is above in the sector: w+offset
//   w+(|w-opttop-boatdir|)
float upLeft(float b, float w){
  float offset = fabsf(w-optPolarTop-b);
  tailAngle=w+offset;
  return tailAngle;
  // sailAngle=tailAngle-angleofattack;
}

// facing left, angle is below in the sector: w+offset
//   w+(|w+180-optboat-boatdir|)
float downLeft(float b, float w){
  float offset = fabsf(w+180-optPolarBot-b);
  tailAngle=w+offset;
  return tailAngle;
  // sailAngle=tailAngle-angleofattack;
}
// facing right, angle is below in the sector: w-offset
//   w-(|w+180+optbot-boatdir|)
float downRight(float b, float w){
  float offset = fabsf(w+180+optPolarBot-b);
  tailAngle=w-offset;
  return tailAngle;
  // sailAngle=tailAngle+angleofattack;
}

// float* buoyRounding(coord_t b, coord_t wp1, coord_t wp2, float initial_anglewaypoint, float normr){

//   float anglewaypoint=initial_anglewaypoint;
//   // get the rotation of a waypoint to be clockwise or not
//   bool clockwise=true;

//   // determining rounding angle
//   float buoy_angle=angleToTarget(wp1.lati,wp1.longi, wp2.lati,wp2.longi);
//   float rounding_angle= -180+(anglewaypoint-buoy_angle);
//   // ccw need ccw angle
//   // if(clockwise){
//   //   rounding_angle=360-rounding_angle;
//   // }
//   rounding_angle=convertto360(rounding_angle);

//   // determining perpindicular distance
//   // pd is the perpindicular distance from the line through b and wp2, to wp1
//   float angle_buoy2=angleToTarget(b.lati,b.longi,wp2.lati,wp2.longi);
//   float wp0_b_wp1_angle=anglewaypoint+angle_buoy2;
//   float pd=normr*sin(180-(wp0_b_wp1_angle+90));

//   return float[2]={rounding_angle,pd};
// }

/*------------------------------------------*/

/*Returns angle (with respect to North) between two global coordinates.*/
float angleToTarget(float lat1, float long1, float lat2, float long2){
  lat1=lat1 * M_PI/180;
  lat2=lat2 * M_PI/180;
  float dLong=(long2-long1) * M_PI/180;
  float y = sin(dLong) * cos(lat2);
  float x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dLong);
  float brng = atan2(y, x) * 180/ M_PI;
  if (brng<0){
    brng+=360;
  }
  return brng;
}

/*Returns great circle distance (in meters) between two global coordinates.*/
double havDist(coord_t  first, coord_t second) {
  double x = first.longitude;
  double y = first.latitude;

  double x1 = second.longitude;
  double y1 = second.latitude;

  const double conversion = M_PI / 180;// term to convert from degrees to radians
  const double r = 6371.0;//radius of earth in km
  x = x * conversion;// convert x to radians
  y = y * conversion;// convert y to radians
  x1 = x1 * conversion;
  y1 = y1 * conversion;

  double half1 = (y-y1) / 2;
  double half2 = (x-x1) / 2;

  double part1 = sin(half1) * sin(half1) + cos(y) * cos(y1) * sin(half2) * sin(half2);
  double part2 = sqrt(part1);
  double distance = 2 * r * asin(part2);// distance is in km due to units of earth's radius

  distance = (distance*1000);

  return distance;
}

// converts an angle to a 0-360 range
float convertto360(float angle){
  angle=(float)((int)angle%360);
  angle=angle+360;
  angle=(float)((int)angle%360);
  return angle;
}
