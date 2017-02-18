#include <Arduino.h>
#include <math.h>
#include "sensors.h"
#include "navigation.h"

void avoidObject(void) {  
   addObjects();
   if (objectVals[1] != 400.0 && objectVals[0] != 400.0) {
    double initialReading = objectVals[1];
    double recentReading = objectVals[0];
    double courseChange = initialReading - recentReading;
    recentReading = (recentReading / 159.5) - 1.0; // this makes
    // recentReading from -1.0 to 1.0 with 0.0 being center of the frame
    if (abs(courseChange) < 0.1) {
      // we need to make an evasion manuever
      if (recentReading > 0)
        recentReading = 1 - recentReading;// reverse recentReading measure
          // so that closer to 1 = closer to center
      else
        recentReading = -1 - recentReading;
      sailAngle += recentReading * 45;
      tailAngle += recentReading * 45;
    }
    else if (initialReading > recentReading) {
      // make starboard turn
      recentReading = abs(45.0*recentReading);
      sailAngle += recentReading;
      tailAngle += recentReading;
    }
    else {
      // make port side turn
      recentReading = abs(45.0*recentReading);
      sailAngle -= recentReading;
      tailAngle -= recentReading;
    }
  }
  
  tailAngle = (float)((int)tailAngle%360);
  tailAngle = tailAngle + 360;
  tailAngle = (float)((int)tailAngle%360);

  sailAngle = (float)((int)sailAngle%360);
  sailAngle = sailAngle+360;
  sailAngle = (float)((int)sailAngle%360);
}

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
