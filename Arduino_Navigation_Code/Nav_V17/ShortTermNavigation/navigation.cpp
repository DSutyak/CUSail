#include <Arduino.h>
#include <math.h>
#include "print.h"
#include "sensors.h"
#include "navigation.h"
#include "navigation_helper.h"
#include "coordinates.cpp"
#include <String>

using namespace std;

Navigation_Controller nc;
Boat_Controller bc;
coord_xy waypoint_array[];

void initializer(){
  pinMode(34, OUTPUT);
  coord_t engQuadRight = {42.4444792, -76.483244}; //Middle of the right sector, looking North, of the engineering quad
  coord_t outsideDuffield = {42.444254, -76.482660}; //Outside West entrance to Duffield Hall, atop the stairs
  coord_t outsideThurston = {42.444228, -76.483666}; //In front of Thurston Hall
// set max distance from origin
  float max_distance = 10000.0;
//  init waypoints
  coord_t coordinates[3] = {outsideDuffield, outsideThurston, engQuadRight};
  int num_wp = 3;
  setOrigin(coordinates[0]);
  delay(1000);
  waypoint_array[num_wp];
  for(int i =0; i < sizeof(coordinates); i++) {
    waypoint_array[i] = xyPoint(coordinates[i]);
  }
  nc.nav_init(max_distance, num_wp, waypoint_array, 10.0, 10.0);
  bc.boat_init(5.0, tailServoPin, sailServoPin);
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

void calcIntendedAngle(Boat_Controller bc, Navigation_Controller nc) {
  if (bc.point_of_sail != "Upwind" && bc.point_of_sail != "Reach" && bc.point_of_sail != "Downwind" ) {
    Serial1.print("Invalid argument sent to nav");
  }
  else if (bc.point_of_sail == "Upwind") {
    if (nc.port_or_starboard == "Port") {
      Serial1.println("Quadrant: UPWIND PORT");
      nc.intended_angle = nc.wind_direction - bc.optimal_angle;
      bc.angle_of_attack = -15;
    }
    else {
      Serial1.println("Quadrant: UPWIND STARBOARD");
      nc.intended_angle= nc.wind_direction + bc.optimal_angle;
      bc.angle_of_attack = 15;
    }
  }
  else if (bc.point_of_sail == "Reach") {
    if (nc.port_or_starboard == "Port") {
      Serial1.println("Quadrant: REACH PORT");
      nc.intended_angle = nc.angle_to_waypoint;
      bc.angle_of_attack = -15;
    }
    else {
      Serial1.println("Quadrant: REACH STARBOARD");
      nc.intended_angle = nc.angle_to_waypoint;
      bc.angle_of_attack = 15;
    }
  }
  else{
     if(nc.port_or_starboard == "Port") {
       Serial1.println("Quadrant: DOWNWIND PORT");
       nc.intended_angle = nc.wind_direction + 180 + bc.optimal_angle;
       bc.angle_of_attack = -15;
     }
     else{
       Serial1.println("Quadrant: DOWNWIND STARBOARD");
       nc.intended_angle = nc.wind_direction + 180 - bc.optimal_angle;
       bc.angle_of_attack = 15;
     }
  }
}


/*
Helper function used to determine whether the boat is aboveBounds or belowBounds.
Arguments:
  width is a float that sets the maximum allowable width for boat to sail within

  point1 and point2 are coordinates in the xy plane
*/

bool aboveBounds(Boat_Controller bc, Navigation_Controller nc){
  float slope = xySlope(bc.location, waypoint_array[nc.current_wp+1]);
  float intercept = bc.location.y - slope * bc.location.x;
  float distance = -1*(slope * bc.location.x - bc.location.y + intercept)/sqrtf(intercept*intercept+1);
  return (distance > nc.upper_width);
}
/*Method to determine whether the boat is below the lesser tacking bound, for use in nShort to determine when to tack */
bool belowBounds(Boat_Controller bc, Navigation_Controller nc){
  float slope = xySlope(bc.location, waypoint_array[nc.current_wp+1]);
  float intercept = bc.location.y - slope * bc.location.x;
  float distance = (slope * bc.location.x - bc.location.y + intercept)/sqrtf(intercept*intercept+1);
  return (distance > nc.lower_width);
}

void nav() {
    nc.wind_direction = sensorData.wind_dir;
    bc.boat_direction = sensorData.boat_direction;
    coord_t coord_lat_lon = {sensorData.x, sensorData.y};
    coord_xy currentPosition = xyPoint(coord_lat_lon);
    bc.location = currentPosition;
    nc.normal_distance = xyDist(nc.waypoint_array[nc.current_wp], bc.location);
    calcIntendedAngle(bc, nc);
    if (bc.detection_radius >= nc.normal_distance) {
      if (nc.current_wp != nc.num_wp) {
        nc.current_wp++;
        if ((int)(bc.boat_direction - nc.wind_direction) % 360 < 180) {
          nc.port_or_starboard = "Port";
        }
        else {
          nc.port_or_starboard = "Starboard";
        }
      }
    }

  if(nc.current_wp != 0 && bc.point_of_sail!= "Reach" && aboveBounds(bc, nc)){
    Serial1.print("HIT UPPER BOUND, TACK RIGHT");
    if(!bc.is_tacking){
      if (nc.port_or_starboard == "Port") {
        nc.port_or_starboard = "Starboard";
      }
      else {
        nc.port_or_starboard = "Port";
      }
    }
    bc.is_tacking = true;
  }
  //Boat hits lower bound, tack left
  else if(nc.current_wp != 0 && bc.point_of_sail!= "Reach" && belowBounds(bc, nc)){
    Serial1.print("HIT LOWER BOUND, TACK LEFT");
    if(!bc.is_tacking){
      if (nc.port_or_starboard == "Port") {
        nc.port_or_starboard = "Starboard";
      }
      else {
        nc.port_or_starboard = "Port";
      }
    }
    bc.is_tacking = true;
  }
  bc.tail_angle = nc.wind_direction + nc.offset;
  bc.sail_angle = bc.tail_angle + nc.intended_angle;

  bc.tail_angle = (float)((int)bc.tail_angle%360);
  bc.tail_angle = bc.tail_angle + 360;
  bc.tail_angle = (float)((int)bc.tail_angle%360);

  bc.sail_angle = (float)((int)bc.sail_angle%360);
  bc.sail_angle = bc.sail_angle+360;
  bc.sail_angle = (float)((int)bc.sail_angle%360);

  //Convert sail and tail from wrt north to wrt boat
  bc.sail_angle = bc.sail_angle - sensorData.boat_direction;
  bc.tail_angle = bc.tail_angle - sensorData.boat_direction;

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

  printData();
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
