#include <math.h>
#include "sensors.h"
#include "coordinates.h"

coord_t origin;
double latOffset;
double longOffset;
double longScale;
const int latToMeter = 111318; //Conversion factor from latitude/longitude to meters

/*Creates origin for XY plane and scales to meters*/
void setOrigin(coord_t startPoint){
    origin.latitude = (double) 0;
    origin.longitude = (double) 0;
    longOffset = startPoint.longitude; //used to generate X coordinate
    latOffset = startPoint.latitude; //used to generate Y coordinate
    longScale = cos(latOffset * M_PI/180);  //scaling factor to account for changing distance between longitude lines
}

/*Converts coordinate in latitude and longitude to xy*/
coord_xy xyPoint(coord_t latlong){
    double x = (latlong.longitude - longOffset) * longScale * latToMeter;
    double y = (latlong.latitude - latOffset) * latToMeter;
    
    coord_xy pt;
    pt.x = x;
    pt.y = y;
    return pt;
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
    
    coord_xy pt;
    pt.x = pointx;
    pt.y = pointy;
    return pt;
}

// converts an angle to a 0-360 range
float convertto360(float angle){
    return (float)((((int)angle%360)+360)%360);
}

// finds closest way point to current location given array of way points
coord_xy find_closest_waypoint(coord_xy c, coord_xy waypoints[]) {
    double min_distance = xyDist(c, waypoints[0]);
    coord_xy min_waypoint = waypoints[0];
    int i;
    for(i = 1; i < sizeof(waypoints) / sizeof(coord_xy); i++) {
        double dist = xyDist(c, waypoints[i]);
        if(dist < min_distance) {
            min_distance = dist;
            min_waypoint = waypoints[i];
        }
    }
    return min_waypoint;
}

char* find_point_of_sail (double angle) {
    if (angle < 60)
        return "Close Reach";
    else if (angle < 120)
        return "Beam Reach";
    else if (angle < 160)
        return "Broad Reach";
    else if (angle < 220)
        return "Dead Run";
    else if (angle < 260)
        return "Broad Reach";
    else if (angle < 300)
        return "Beam Reach";
    else
        return "Close Reach";
}

char* find_tack (double angle) {
    if (angle < 180)
        return "Port";
    else
        return "Starboard";
}

double calculate_vmg (double angleToTarget, double boatSpeed) {
    return boatSpeed * cos(angleToTarget);
}

double true_wind (double windAngle, double windSpeed, double boatSpeed) {
    return -acos(((windSpeed * cos(windAngle) - boatSpeed) / 
            sqrt(windSpeed * windSpeed + boatSpeed * boatSpeed + 2 * windSpeed * boatSpeed * cos(windAngle))));
}

coord_xy diff (coord_xy T, coord_xy B) {
    coord_xy pt;
    pt.x = T.x - B.x;
    pt.y = T.y - B.y;
    return pt;
}

double fPolar (double windSpeed, double angle) {
    return 0; // TODO: Polar diagram
}