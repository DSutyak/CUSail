#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include "sensors.h"
#include "coordinates.h"

//a struct containing the information on any loops we want in our waypoints
typedef struct {
    //index of the waypoint at which the loop begins
    int start;
    //index of the waypoint at which the loop ends
    int end;
    //pointer to a function that returns true if we loop and false if not
    //must take no arguments and return a boolean
    bool (*condition)(void);
} loop;

//a struct to generate a linked list of loops in the case we need multiple
typedef struct{
    loop data;
    struct loop_node *next;
} loop_node;

coord_xy origin;
double latOffset;
double longOffset;
double longScale;
double detectionRadius = 5.0;
const int latToMeter = 111318; //Conversion factor from latitude/longitude to meters
const int radEarth = 6371000;

//change this based on the number of waypoints you have
int waypointTotal = 2;
coord_t *rawWaypoints;
coord_xy *waypoints;
int currentWaypoint;
loop_node waypoint_loops = {{NULL, NULL, NULL}, NULL};

/*Creates origin for XY plane and scales to meters*/
void setOrigin(coord_t startPoint){
    origin.x = (double) 0;
    origin.y = (double) 0;
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

void navigationInit() {
    rawWaypoints = (coord_t *) malloc(waypointTotal * sizeof(coord_t));
    waypoints = (coord_xy *) malloc(waypointTotal * sizeof(coord_xy));
    rawWaypoints[0].latitude = 42.445457;
    rawWaypoints[0].longitude = -76.484322;
    rawWaypoints[1].latitude = 42.444259;
    rawWaypoints[1].longitude = -76.484435;
    currentWaypoint = 0;
    setOrigin(rawWaypoints[0]);
    int i;
    for(i = 0; i < waypointTotal; i++) {
        waypoints[i] = xyPoint(rawWaypoints[i]); 
    }
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
/*Finds a waypoint given the bouy, the waypoint NOT USED to generate the angles provided, the rounding distance,
 * and the clockwise and counterclockwise angles from the buoy. 
 * Example: If you are generating the first waypoint in rounding a buoy, you will supply the buoy's coordinate,
 * the first waypoint after the buoy is done being rounded, the rounding distance, and the two potential
 * desired angles of entry (the angle of the vector between the preceding waypoint and the buoy +- 90).
 * If you are generating the middle waypoint, supply buoy as target and NULL as angle2.
 */
coord_xy find_rounding_waypoint(coord_xy buoy, coord_xy target, int rounding_dist, double angle1, double angle2){
    //do some trig here
    //hypotenuse = rounding dist
    double dx_clockwise;
    double dy_clockwise;
    double angle1_rad;
    if (((int)angle1)/90 % 2 == 0){
        angle1_rad = angle1 * M_PI / 180.0;
        dx_clockwise = rounding_dist * sin(angle1_rad) * 180.0 /M_PI;
        dy_clockwise = rounding_dist * cos(angle1_rad) * 180.0 /M_PI;
    }
    else{
        angle1_rad = angle1 * M_PI / 180.0;
        dx_clockwise = rounding_dist * cos(angle1_rad) * 180.0 /M_PI;
        dy_clockwise = rounding_dist * sin(angle1_rad) * 180.0 /M_PI;
    }
    coord_xy clockwise_point = {buoy.x+dx_clockwise,buoy.y+dy_clockwise};
    if (angle2 == NULL){
        return clockwise_point;
    }
    
    double angle2_rad;
    double dx_cntrclockwise;
    double dy_cntrclockwise;
    
    if (((int)angle2)/90 % 2 == 0){
        angle2_rad = angle2 * M_PI / 180.0;
        dx_cntrclockwise = rounding_dist * sin(angle2_rad) * 180.0 /M_PI;
        dy_cntrclockwise = rounding_dist * cos(angle2_rad) * 180.0 /M_PI;
    }
    else{
        angle2_rad = angle2 * M_PI / 180.0;
        dx_cntrclockwise = rounding_dist * cos(angle2_rad) * 180.0 /M_PI;
        dy_cntrclockwise = rounding_dist * sin(angle2_rad) * 180.0 /M_PI;
    }
    coord_xy cntrclockwise_point = {buoy.x+dx_cntrclockwise,buoy.y+dy_cntrclockwise};
    //return point on opposite side of target, such that we go around the buoy instead of just getting close
    if (xyDist(clockwise_point, target) >= xyDist(cntrclockwise_point, target)){
        return clockwise_point;
    }
    else{
        return cntrclockwise_point;
    }
}

void round_buoy(coord_xy buoy, coord_xy preceding_wp, coord_xy succeeding_wp, int preceding_idx, int rounding_dist){
    double prec_angle = angleToTarget(preceding_wp, buoy);
    
    double clockwise_start_angle = (prec_angle + 90);
    clockwise_start_angle = clockwise_start_angle >= 360 ? clockwise_start_angle - 360 : clockwise_start_angle;
    
    double cntrclockwise_start_angle = (prec_angle - 90);
    cntrclockwise_start_angle = cntrclockwise_start_angle < 0 ? clockwise_start_angle + 360 : cntrclockwise_start_angle;
    
    coord_xy first_point = find_rounding_waypoint(buoy, succeeding_wp, rounding_dist, clockwise_start_angle, cntrclockwise_start_angle);
    
    double succ_angle = angleToTarget(buoy, succeeding_wp);
    
    double clockwise_end_angle = (succ_angle + 90);
    clockwise_end_angle = clockwise_end_angle >= 360 ? clockwise_end_angle - 360 : clockwise_end_angle;
    
    double cntrclockwise_end_angle = (succ_angle - 90);
    cntrclockwise_end_angle = cntrclockwise_end_angle < 0 ? clockwise_end_angle + 360 : cntrclockwise_end_angle;
    
    coord_xy last_point = find_rounding_waypoint(buoy, succeeding_wp, rounding_dist, clockwise_end_angle, cntrclockwise_end_angle);
    
    double midpoint_angle = (angleToTarget(buoy, first_point) + angleToTarget(buoy, last_point))/2;
    coord_xy mid_point = find_rounding_waypoint(buoy, buoy, rounding_dist, midpoint_angle, NULL);
    
    //realloc waypoint array with 3 points following preceding_idx
    waypointTotal+=3;
    coord_xy *new_waypoint_array = (coord_xy *) malloc((waypointTotal) * sizeof(coord_xy));
    int i;
    for (i = 0; i < waypointTotal; i++){
        new_waypoint_array[i] = waypoints[i];
        if (i == preceding_idx){
            new_waypoint_array[i+1] = first_point;
            new_waypoint_array[i+2] = mid_point;
            new_waypoint_array[i+3] = last_point;
        }
    }
    loop_node *front = &waypoint_loops;
    do{
        if (front->data.start <= preceding_idx && front->data.end > preceding_idx){
            front->data.end += 3;
        }
        front = front->next;
    } while(front != NULL);
    
    free(waypoints);
    waypoints = new_waypoint_array;
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
    coord_xy targetPosition = waypoints[currentWaypoint]; // initialize T
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

void add_waypoint_loop(int start, int end, bool (*condition)(void)){
    if (waypoint_loops.data.start == NULL){
        waypoint_loops.data.start = start;
        waypoint_loops.data.end = end;
        waypoint_loops.data.condition = condition;
        waypoint_loops.next = NULL;
    }
//    TODO: evaluate whether or not the memory stuff here actually works
    else {
        loop_node *front = &waypoint_loops;
        while(front->next != NULL){
            front = front->next;
        }
        loop_node next_loop = {{start, end, condition}, NULL};
        front->next = &next_loop;
    }
}

int check_loops(){
    if(waypoint_loops.data.start == NULL){
        return (currentWaypoint+1 == waypointTotal)? currentWaypoint : currentWaypoint + 1;
    }
    
    loop_node *front = &waypoint_loops;
    do{
        if(currentWaypoint == front->data.end){
            if (front->data.condition()){
                return front->data.start;
            }
        }
        front = front->next;
    }while(front != NULL);
    
    return (currentWaypoint+1 == waypointTotal)? currentWaypoint : currentWaypoint + 1;
}

void nav() {
    coord_xy pos = {sensorData->x, sensorData->y};
    if(xyDist(pos, waypoints[currentWaypoint]) < detectionRadius) {
        currentWaypoint = check_loops();
    }
    double nextAngle = calculateAngle();
    setServoAngles(nextAngle);
}