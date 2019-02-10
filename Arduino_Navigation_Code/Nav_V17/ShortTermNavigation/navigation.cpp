#include <Arduino.h>
#include <math.h>
#include "print.h"
#include "sensors.h"
#include "navigation.h"
#include "navigation_helper.h"
#include "coordinates.cpp"
#include <String.h>
#include <iostream>
#include <array>

using namespace std;

void initializer(void){


  /*
  Constructor for a boat. Sets up the servos, and establishes intial values
  for each boat variable.

  Arguments:
    detection_radius is a float that represents how close we have to
    be to a waypoint to mark it as 'hit'. Precondition: detection_radius must
    be less than the upper and lower tacking bounds.
  */
  Boat_Controller::Boat_Controller (float d) {
    set_sail_angle(0.00);
    set_tail_angle(0.00);
    boat_direction = sensorData.boatDir;
    tailServo.attach(tailServoPin);
    sailServo.attach(sailServoPin);
    detection_radius = d;
    isTacking = false;
    PointofSail = "";
    initSensors();
  }  //Sets the angle of the main sail


/*
An object of class Navigation_Controller represents the abstract
(not directly related to the boat) variables and operations performed on them to
navigate a body of water.
*/

  /*
  Constructor for a Navigation Controller.
  Sets up the waypoints, and establishes
  default values for each navigation variable.

  Arguments:
    max is a float that represents how far we can get from the origin before
    the origin must be reset.

    num is an int that represents the number of coord_xy waypoints in the
    array waypoints argument.

    port and starboard are floats that represent
    how far (in meters) to port and starboard the boat is allowed to
    go before tacking.
  */
  Navigation_Controller::Navigation_Controller (float limit, int num, coord_xy waypoints[], float port, float starboard){
    int count = 0;
    while (count < num){
      waypoint_array[count] = waypoints[count];
      count++;
    }
    angleToWaypoint = 0.0;
    normalDistance = 0.0;
    intendedAngle = 0.0;
    portOrStarboard = "";
    maxDistance = limit;
    numWP = num;
    currentWP = 0;
    dirAngle = 0.0;
    offset = 0.0;
    wind_direction = 0.0;
    port_boundary = port;
    starboard_boundary = starboard;
    upperWidth = 10;
    lowerWidth = 10;
  }

  coord_t [3] = {outsideDuffield, outsideThurston, engQuadRight};
  set_origin([0]);
  for(int i =0; i < sizeof(); i++){
    [i] = xyPoint(coordinates[i]);
  }
  
  float maxDistance = 10000.0;
  int numWp = 3;
  coord_xy waypoint_array = coordinates;
  port_boundary = 10.0;
  starboard_boundary = 10.0;
  Navigation_Controller nc = Navigation_Controller(maxDistance, numWP, waypoint_array, port_boundary, starboard_boundary);
  Boat_Controller bc = Boat_Controller(5.0);
}

  // nav must be called after the initializer
void nav(void) {

    nc.wind_direction = sensorData.windDir;
    bc.boat_direction = sensorData.boatDir;
    r[0] = nc.waypoint_array[nc.numWP].x - sensorData.x;
    r[1] = nc.waypoint_array[nc.numWP].y - sensorData.y;
    w[0] = cos((sensorData.windDir)*(PI/180.0));
    w[1] = sin((sensorData.windDir)*(PI/180.0));
    coord_t coord_lat_lon = {sensorData.x, sensorData.y};
    coord_xy currentPosition = xyPoint(coord_lat_lon);
    bc.location = currentPosition;
    bc.normalDistance = xyDist(nc.waypoint_array[nc.currentWP], bc.currentPosition);
    calcIntendedAngle(bc, nc);
    if (bc.detection_radius >= nc.normalDistance) {
      if (nc.currentWP != nc.numWP) {
        nc.currentWP++;
        if ((bc.boat_direction - nc.wind_direction) % 360 < 180) {
          nc.portOrStarboard = "Port";
        }
        else {
          nc.portOrStarboard = "Starboard";
        }
      }
    }
  }
  if(nc.currentWP != 0 && bc.pointofSail!=1 && aboveBounds(nc.upperWidth, bc.location, nc.waypoint_array[currentWP+1], bc.PointofSail)){
    Serial1.print("HIT UPPER BOUND, TACK RIGHT");
    if(!bc.isTacking){
      if (nc.portOrStarboard == "Port") {
        nc.portOrStarboard = "Starboard";
      }
      else {
        nc.portOrStarboard = "Port";
      }
    }
    bc.isTacking = true;
  }
  //Boat hits lower bound, tack left
  else if(wpNum != 0 && pointofSail!=1 && belowBounds(nc.lowerWidth, bc.location, nc.waypoint_array[nc.currentWP+1], bc.PointofSail) ){
    Serial1.print("HIT LOWER BOUND, TACK LEFT");
    if(!bc.isTacking){
      if (nc.portOrStarboard == "Port") {
        nc.portOrStarboard = "Starboard";
      }
      else {
        nc.portOrStarboard = "Port";
      }
    }
    bc.isTacking = true;
  }
  bc.tail_angle = nc.wind_direction + nc.offset;
  bc.sail_angle = bc.tail_angle + nc.intendedAngle;

  bc.tail_angle = (float)((int)bc.tail_angle%360);
  bc.tail_angle = bc.tail_angle + 360;
  bc.tail_angle = (float)((int)bc.tail_angle%360);

  bc.sail_angle = (float)((int)bc.sail_angle%360);
  bc.sail_angle = bc.sail_angle+360;
  bc.sail_angle = (float)((int)bc.sail_angle%360);

  //Convert sail and tail from wrt north to wrt boat
  bc.sail_angle = bc.sail_angle - sensorData.boatDir;
  bc.tail_angle = bc.tail_angle - sensorData.boatDir;

  // convert sail to 0-360
  bc.sail_angle = int(bc.sail_angle+360)%360;

  // convert tail to -180-180
  bc.tail_angle = int(bc.tail_angle+360)%360;
  if (bc.tail_angle> 180) {bc.tail_angle -= 360;}

  //Get servo commands from the calculated sail and tail angles
  if (bc.sail_angle < 0) {
    bc.sail_angle += 360;
  }

  bc.set_sail_angle(bc.sail_angle);
  bc.set_tail_angle(bc.tail_angle);

}

/*
Helper function used to calculate intendedAngle.
Determines if the boat is upwind, downwind, or reach, and if
port or starboard, then determines intendedAngle and
angle_of_attack.
Arguments:
  pointOfSail is a string that represents upwind, downwind, or reach (direct)
  portOrStarboard is a string that represents port (left) or starboard (right)
*/
public void calcIntendedAngle(Boat_Controller bc, Navigation_Controller nc) {
  if (bc.PointofSail != "Upwind" && nc.PointofSail != "Reach" && nc.PointofSail != "Downwind" ) {
    Serial1.print("Invalid argument sent to nav");
  }
  else if (nc.PointofSail == "Upwind") {
    if (bc.portOrStarboard == "Port") {
      Serial1.println("Quadrant: UPWIND PORT");
      this->nc.intendedAngle = nc.wind_direction - optimal_angle;
      this.nc.angle_of_attack = -15;
    }
    else {
      Serial1.println("Quadrant: UPWIND STARBOARD");
      nc.intendedAngle= nc.wind_direction + optimal_angle;
      nc.angle_of_attack = 15;
    }
  }
  else if (nc.PointofSail == "Reach") {
    if (bc.portOrStarboard == "Port") {
      Serial1.println("Quadrant: REACH PORT");
      nc.intendedAngle = angleToWaypoint;
      nc.angle_of_attack = -15;
    }
    else {
      Serial1.println("Quadrant: REACH STARBOARD");
      nc.intendedAngle = angleToWaypoint;
      nc.angle_of_attack = 15;
    }
  }
  else{
     if(bc.portOrStarboard == "Port") {
       Serial1.println("Quadrant: DOWNWIND PORT");
       nc.intendedAngle = windDir + 180 + optimal_angle;
       nc.intended_angle_of_attack = -15;
     }
     else{
       Serial1.println("Quadrant: DOWNWIND STARBOARD");
       nc.intendedAngle = nc.wind_direction + 180 - optimal_angle;
       nc.intended_angle_of_attack = 15;
     }
  }
}
/*
Helper function used to determine whether the boat is aboveBounds or belowBounds.
Arguments:
  width is a float that sets the maximum allowable width for boat to sail within

  point1 and point2 are coordinates in the xy plane
*/

public bool aboveBounds(Navigation_Controller nc, Boat_Controller bc){
  float slope = xySlope(bc.locatation, waypoint_array[nc.currentWP+1]);
  float intercept = bc.location.y - slope * bc.location.x;
  float distance = -1*(slope * bc.location.x - bc.location.y + intercept)/sqrtf(intercept*intercept+1);
  return (distance > nc.upperWidth);
}
/*Method to determine whether the boat is below the lesser tacking bound, for use in nShort to determine when to tack */
public bool belowBounds(Navigation_Controller nc, Boat_Controller bc){
  float slope = xySlope(bc.location, waypoint_array[nc.currentWP+1]);
  float intercept = bc.location.y - slope * bc.location.x;
  float distance = (slope * bc.location.x - bc.location.y + intercept)/sqrtf(intercept*intercept+1);
  return (distance > nc.lowerWidth);
}

/*void endurance(coord_xy buoyLocations[]){
  if(nc.currentWP = buoyLocations.length){
    nc.currentWP = 0;
  }
  nc.maxDistance = 3;
  nc.waypoint_array = buoyLocations;
  nav();

}
// 5 Meters away from the Buoy inside the buoy location. Must hand calculate the
// Buoy Location
void station_keeping(coord_xy buoyLocations[]){
  if(true){
    coord_xy next = nc.waypoint_array[(nc.currentWP + 1)%4];
    coord_xy current = bc.location;
  }
  if(nc.currentWP = buoyLocations.length){
    nc.currentWP = 0;
  nc.maxDistance = 3;
  nc.waypoint_array = buoyLocations;
  nav()
}

// Once again, always be 5 meters outside bouy, must be within 3 feet of buoy
// to avoid hitting buoy
// Alternatively, set distance to .75m for the whole thing. Set wayPoints to
//  be 2 meters away from buoys.
// Steps: 1. Sail to waypoint about 5 meters in front of buoys
// 2. Sail to waypoint adjacent to first bouy, 5 meters outside it
// 3. Sail to waypoint "below" first buoy, 5 meters away from it
// 4. Sail to waypoint "below" second buoy ~5 meters
// 5. Sail to waypoint adjecent to second buoy, ~5 meters outside
// 6. Sail to waypoints ~5 meters in front of the double buoys
// 7. Sail to waypoint directly between buoys. Must be within .75 meters
// 8. Sail to waypoint behind buoys. Must be within .75 meters.
bool waypointsSet = false
void precision_nav(coord_xy buoyLocations[]){
  if (not waypointsSet) {
    coord_xy w1 =
    coord_xy w2 =
    coord_xy w3 =
    coord_xy w4 =
    coord_xy w5 =
    coord_xy w6 =
    coord_xy w7 = middlePoint(buoyLocation[0], buoyLocation[1])
    coord_xy w8 =
  }


}


// Ints corresponding to the different events
int endurance = 1;
int station_keeping = 2;
int precision = 3;
int payload = 4;
int collison = 5;
int search = 6;
int current = 1;

// Offset the buoy locations by hand. Translate to xy etc. buoy locations coord_xy
// to be 5 meters inside the buoy
coord_xy buoyLocations[] = {};
float accSeconds = 0;

void nShort(void){
  switch (current){
    case endurance: endurance(buoyLocations)
    case station_keeping: station_keeping(buoyLocations)
    case precision:
    case payload:
    case collision:
    case search:
  }
}
*/
