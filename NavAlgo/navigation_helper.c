#include <math.h>
#include "sensors.h"
#include "coordinates.h"

coord_xy waypoints[];

coord_t origin;
double latOffset;
double longOffset;
double longScale;
const int latToMeter = 111318; //Conversion factor from latitude/longitude to meters
const int radEarth = 6371000;


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
/*This function calculate (x,y) as defined in the paper "Autonomous Sailboat Navigation for Short Course Racing"*/
coord_xy xyPoint2(coord_t latlong){
    double x = radEarth * cos(latlong.latitude - latOffset) * M_PI/180 * (latlong.longitude - longOffset);
    double y = radEarth * M_PI/180 * (latlong.latitude - latOffset);
    
    coord_xy pt2;
    pt2.x = x;
    pt2.y = y;
    return pt2;
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
/*For angles between 43 and 151 degrees
 the constant "1.397" was calculated by taking the average over the interval
 * -pi/2 to pi/2 of the integral of the normalized polar function
 defined in "Autonomous Sailboat Navigation for Short Course Racing"
 We used their numbers, so these values are NOT empirically optimized for our boat*/
double fPolar (double windSpeed, double angle) {
    if (43 < angle < 151 || 209 < angle < 317)
         return windSpeed * 1.397 ; // TODO: Polar diagram
    else 
        return 0;
}

/*
 * calculateAngle() calculates the optimal boat direction towards the nearest waypoint 
 * given wind speed, wind direction, position, direction, waypoint positions
 * TODO: check logic, test (a lot), integrate obstacle detection
 */
double calculateAngle() {
    // initializing variables for calculations
    coord_xy boatPosition = {sensorData->x, sensorData->y}; // initialize B
    coord_xy targetPosition = find_closest_waypoint(boatPosition, waypoints); // initialize T
    coord_xy boatTargetDifference = diff (targetPosition, boatPosition); // initialize t
    double t_mag = xyDist(boatPosition, targetPosition); // initialize magnitude of t
    double boat_heading = sensorData->boat_direction; // initialize phi(b))
    double beating_param = 10; // this can be adjusted
    double windDirection = sensorData->wind_dir; // should probably use true wind
    double intendedAngle = angleToTarget(boatPosition, targetPosition);
    double angleDifference = (double)(((((int)(windDirection - intendedAngle)) % 360) + 360) % 360); // finds positive angle between wind and intended path
    double hysteresis = 1 + (beating_param / t_mag); // initialize n
    double inverseWindAngle = angleDifference; // TODO: initialize phi(-w)?
    double alpha = 0.0; 
    double v_maxR = 0.0;
    double v_maxL = 0.0;
    double phi_bmaxR = inverseWindAngle;
    double phi_bmaxL = inverseWindAngle;
    double v_hyp;
    double v_tR;
    double v_tL;
    double phi_bnew;
    double delta_alpha = 5.0; // can change this
    while (alpha < 180) {
        v_hyp = fPolar (sensorData->wind_speed, (inverseWindAngle + alpha));
        v_tR = abs(v_hyp * cos((double)(((int)(inverseWindAngle + alpha))%360))); // Is this right
        if (v_tR > v_maxR) {
            v_maxR = v_tR;
            phi_bmaxR = (double)(((int)(inverseWindAngle + alpha))%360);
        }
        alpha = alpha + delta_alpha;
    }
    alpha = 0;
    while (alpha < 180) {
        v_hyp = fPolar (sensorData->wind_speed, (inverseWindAngle - alpha));
        v_tL = abs(v_hyp * cos((double)(((((int)(inverseWindAngle - alpha))%360)+360)%360))); // Is this right part 2
        if (v_tL > v_maxL) {
            v_maxL = v_tL;
            phi_bmaxL = (double)(((((int)(inverseWindAngle - alpha))%360)+360)%360);
        }
        alpha = alpha + delta_alpha;
    }
    if (abs((int)(phi_bmaxR - boat_heading)) < abs((int)(phi_bmaxL - boat_heading))) {
        if(v_maxR * hysteresis < v_maxL) {
            phi_bnew = phi_bmaxL;
        }
        else {
            phi_bnew = phi_bmaxR;
        }
    }
    else {
        if(v_maxL * hysteresis < v_maxR) {
            phi_bnew = phi_bmaxR;
        }
        else {
            phi_bnew = phi_bmaxL;
        }
    }
    // phi_bnew is angle w.r.t. wind, so needs to be converted to w.r.t. north
    return (double)((((int)(phi_bnew + sensorData->wind_dir) % 360) + 360) % 360);
}

/*
 * setServoAngles sets the servo angles according to angleToSail. This is mostly
 * copied from previous algorithm.
 * TODO: check logic, test
 */
void setServoAngles(double angleToSail) {
  
    // calculate angle of attack for sail angle
    double angleOfAttack;
    if(sensorData->wind_dir < 180) // based on previous algorithm and also Wikipedia, 15 degrees is critical angle of attack
        angleOfAttack = -15;
    else
        angleOfAttack = 15;
    
    double offset = sensorData->boat_direction - angleToSail;
    double tail_angle = sensorData->wind_dir + offset;
    double sail_angle = tail_angle + angleOfAttack;

    tail_angle = (double)((((int)tail_angle%360)+360)%360);
    sail_angle = (double)((((int)sail_angle%360)+360)%360);

    //Convert sail and tail from wrt north to wrt boat
    sail_angle = sail_angle - sensorData->boat_direction;
    tail_angle = tail_angle - sensorData->boat_direction;

    // convert sail to 0-360
    sail_angle = (double)((((int)sail_angle%360)+360)%360);

    // convert tail to -180-180
    tail_angle = (double)((((int)tail_angle%360)+360)%360);
    while (tail_angle> 180) {tail_angle -= 180;}

    sensorData->sailAngleBoat = sail_angle;
    sensorData->tailAngleBoat = tail_angle;

    setSailServoAngle(sail_angle);
    setTailServoAngle(sail_angle, tail_angle);
}
