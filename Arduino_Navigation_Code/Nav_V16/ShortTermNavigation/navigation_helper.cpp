#include <Arduino.h>
#include <math.h>
#include "sensors.h"
#include "navigation.h"


coord_xy origin;
double latOffset;
double longOffset;
double longScale;
const int latToMeter = 111318; //Conversion factor from latitude/longitude to meters

/*Creates origin for XY plane and scales to meters*/
void setOrigin(coord_t startPoint){
  origin = new coord_xy((double) 0, (double) 0);
  longOffset = startPoint.longitude; //used to generate X coordinate
  latOffset = startPoint.latitude; //used to generate Y coodinate
  longScale = cos(latOffset * M_PI/180);  //scaling factor to account for changing distance between longitude lines
}

/*Converts coordinate in latitude and longitude to xy*/
coord_xy xyPoint(coord_t latlong){
  double x = (latlong.longitude - longOffset) * longScale * latToMeter;
  double y = (latlong.latitude - latOffset) * latToMeter;

  return new coord_xy(x, y);
}

/*finds the distance between two xy points*/
double xyDist(coord_xy point1, coord_xy point2){
  double dx = point1.x - point2.x;
  double dy = point1.y - point2.y;
  return sqrt(dx * dx + dy * dy);
}

/*Returns angle (with respect to North) between two global coordinates.*/
float angleToTarget(coord_xy coord1, coord_xy coord2){
  double dx = fabs(coord1.x-coord2.x);
  double dy = fabs(coord1.y-coord2.y);
  double angle = atan(dx/dy) * 180/M_PI;
  if (angle<0){
    angle+=360;
  }
  return angle;
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
