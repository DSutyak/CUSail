#include <math.h>
#include "sensors.h"
#include "coordinates.c"

coord_t origin;
double latOffset;
double longOffset;
double longScale;
const int latToMeter = 111318; //Conversion factor from latitude/longitude to meters

/*Creates origin for XY plane and scales to meters*/
void setOrigin(coord_t startPoint){
  /* Communication */
  // print("Origin set at ");
  // print(startPoint.latitude);
  // print(" North : ");
  // print(startPoint.longitude);
  // println(" west");
  origin = coord_xy({(double) 0, (double) 0});
  longOffset = startPoint.longitude; //used to generate X coordinate
  latOffset = startPoint.latitude; //used to generate Y coordinate
  longScale = cos(latOffset * M_PI/180);  //scaling factor to account for changing distance between longitude lines
}

/*Converts coordinate in latitude and longitude to xy*/
coord_xy xyPoint(coord_t latlong){
  double x = (latlong.longitude - longOffset) * longScale * latToMeter;
  double y = (latlong.latitude - latOffset) * latToMeter;
  return coord_xy({x, y});
}

/*finds the distance between two xy points*/
double xyDist(coord_xy point1, coord_xy point2){
  double dx = point1.x - point2.x;
  double dy = point1.y - point2.y;
  return sqrt(dx * dx + dy * dy);
}

//for distance between current and next waypoint
//return xyDist({sensorData.x,sensorData.y},wayPoints[wpNum]);

/*Returns angle (with respect to North) between two global coordinates.*/
float angleToTarget(coord_xy coord1, coord_xy coord2){
  coord_xy newC1 = {0.0,10.0};
  coord_xy newC2 = {coord2.x - coord1.x, coord2.y - coord1.y};
  double dot = newC1.x * newC2.x + newC1.y * newC2.y;
  double det = newC1.y * newC2.x - newC2.y * newC1.x;
  double angle = atan2(det, dot) * 180/M_PI;
  if (angle<0){
    angle+=360;
  }
  return angle;
}

/*Calculates slope between point1 and point2, designed for use with tacking boundaries */
float xySlope(coord_xy point1, coord_xy point2){
    double dx = point1.x - point2.x;
    double dy = point1.y - point2.y;
    int sign = 1;
    if (fabs(dx) < 0.0000001)
    {
      if(dx < 0){
        sign = -1;
      }
      return sign * 10000000;
    }
    return dy/dx;
}

coord_xy middlePoint(coord_xy point1, coord_xy point2){
  double xdis= (point1.x - point2.x)/2;
  double ydis= (point1.y - point2.y)/2;
  double pointx= point1.x-xdis;
  double pointy = point1.y-ydis;
  return coord_xy({pointx, pointy});
}

// converts an angle to a 0-360 range
float convertto360(float angle){
  return (float)((((int)angle%360)+360)%360);
}