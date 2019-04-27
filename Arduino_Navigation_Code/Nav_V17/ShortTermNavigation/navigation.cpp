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
  coord_t hollister = {42.444259, -76.484435}; //Hollister Hall
  coord_t engQuadX = {42.444612, -76.483492}; //Center of Engineering Quad
  coord_t engQuadRight = {42.4444792, -76.483244}; //Middle of the right sector, looking North, of the engineering quad
  coord_t outsideDuffield = {42.444254, -76.482660}; //Outside West entrance to Duffield Hall, atop the stairs
  coord_t outsideThurston = {42.444228, -76.483666}; //In front of Thurston Hall
// set max distance from origin
  float max_distance = 10000.0;
//  init waypoints
  coord_t coordinates[4] = {engQuadX, outsideThurston, hollister, engQuadRight};
  nc.num_wp = 4;
  setOrigin(coordinates[0]);
  delay(1000);
  waypoint_array[nc.num_wp];
  for(int i =0; i < sizeof(coordinates); i++) {
    waypoint_array[i] = xyPoint(coordinates[i]);
  }
  nc.nav_init(max_distance, nc.num_wp, waypoint_array, 10.0, 10.0);
  bc.boat_init(5.0, sailServoPin, tailServoPin);
  nc.angle_to_waypoint =
    angleToTarget(bc.location, waypoint_array[nc.current_wp]);
}

/*
void stationkeeping_initializer(){
  pinMode(34, OUTPUT);
  these are just placeholders, set these as the bouy locations
  coord_t bouy1 = {42.4444792, -76.483244}; //Middle of the right sector, looking North, of the engineering quad
  coord_t bouy2 = {42.444254, -76.482660}; //Outside West entrance to Duffield Hall, atop the stairs
  coord_t bouy3 = {42.444228, -76.483666}; //In front of Thurston Hall
  coord_t bouy4 = {42.444228, -76.483666}; //In front of Thurston Hall
  center = {(bouy1.latitude+bouy2.latitude+bouy3.latitude+bouy4.latitude)/4,
  (bouy1.longitude+bouy2.longitude+bouy3.longitude+bouy4.longitude)/4};
// set max distance from origin
  float max_distance = 10000.0;
//  init waypoints
  coord_t coordinates[2] = {center, center};
  int num_wp = 2;
  setOrigin(coordinates[0]);
  delay(1000);
  waypoint_array[num_wp];
  for(int i =0; i < sizeof(coordinates); i++) {
    waypoint_array[i] = xyPoint(coordinates[i]);
  }
  nc.nav_init(max_distance, num_wp, waypoint_array, 10.0, 10.0);
  bc.boat_init(5.0, tailServoPin, sailServoPin);
  float angle_to_waypoint =
    angleToTarget(coord_xy({sensorData.x, sensorData.y}), waypoint_array[num_wp]);
}
*/

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
    //Serial1.print("Invalid argument sent to nav");
  }
  else if (bc.point_of_sail == "Upwind") {
    if (nc.port_or_starboard == "Port") {
      //Serial1.println("Quadrant: UPWIND PORT");
      nc.intended_angle = nc.wind_direction - bc.optimal_angle;
      bc.angle_of_attack = -15;
    }
    else {
      //Serial1.println("Quadrant: UPWIND STARBOARD");
      nc.intended_angle= nc.wind_direction + bc.optimal_angle;
      bc.angle_of_attack = 15;
    }
  }
  else if (bc.point_of_sail == "Reach") {
    if (nc.port_or_starboard == "Port") {
      //Serial1.println("Quadrant: REACH PORT");
      nc.intended_angle = nc.angle_to_waypoint;
      bc.angle_of_attack = -15;
    }
    else {
      //Serial1.println("Quadrant: REACH STARBOARD");
      nc.intended_angle = nc.angle_to_waypoint;
      bc.angle_of_attack = 15;
    }
  }
  else{
     if(nc.port_or_starboard == "Port") {
       //Serial1.println("Quadrant: DOWNWIND PORT");
       nc.intended_angle = nc.wind_direction + 180 + bc.optimal_angle;
       bc.angle_of_attack = -15;
     }
     else{
       //Serial1.println("Quadrant: DOWNWIND STARBOARD");
       nc.intended_angle = nc.wind_direction + 180 - bc.optimal_angle;
       bc.angle_of_attack = 15;
     }
  }
}


/*
Helper functions used to determine whether the boat is aboveBounds or belowBounds.
Arguments:
  bc.location is the current GPS xy coordinate of the boat
  waypoint_array[nc.current_wp+1] is the value of the next waypoint the boat must hit
*/

/*function [aboveBounds] determines if the boat is above the greatest tacking bound (port) */
bool aboveBounds(Boat_Controller bc, Navigation_Controller nc){
  float slope = xySlope(bc.location, waypoint_array[nc.current_wp]);
  float intercept = bc.location.y - slope * bc.location.x;
  float distance = -1*(slope * bc.location.x - bc.location.y + intercept)/sqrtf(intercept*intercept+1);
  return (distance > nc.upper_width);
}
/*function [belowBounds] determines if the boat is below the lowest tacking bound (starboard) */
bool belowBounds(Boat_Controller bc, Navigation_Controller nc){
  float slope = xySlope(bc.location, waypoint_array[nc.current_wp]);
  float intercept = bc.location.y - slope * bc.location.x;
  float distance = (slope * bc.location.x - bc.location.y + intercept)/sqrtf(intercept*intercept+1);
  return (distance > nc.lower_width);
}

void nav() {
    bc.boat_direction = convertto360(sensorData.boat_direction);
    nc.wind_direction = convertto360(270.0 - bc.boat_direction);
    bc.location = sensorData.location;
    nc.normal_distance = xyDist(waypoint_array[nc.current_wp], bc.location);
    nc.angle_to_waypoint =
     angleToTarget(bc.location, waypoint_array[nc.current_wp]);
    nc.dir_angle = convertto360(nc.angle_to_waypoint-nc.wind_direction);
    if (bc.detection_radius > nc.normal_distance) {
      if (nc.current_wp != nc.num_wp) {
        nc.current_wp++;
        if ((int) convertto360(bc.boat_direction - nc.wind_direction) < 180) {
          nc.port_or_starboard = "Port";
        }
        else {
          nc.port_or_starboard = "Starboard";
        }
      }
    }

  if(nc.current_wp != 0 && bc.point_of_sail!= "Reach" && aboveBounds(bc, nc)){
//    Serial1.print("HIT UPPER BOUND, TACK RIGHT");
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
//    Serial1.print("HIT LOWER BOUND, TACK LEFT");
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
  // Boat is on a port tack (wind is hitting the sail from the left)
  else if(bc.boat_direction < 180) {
    if (nc.dir_angle<bc.optimal_angle && nc.dir_angle>0){
      bc.point_of_sail = "Upwind";
      nc.port_or_starboard = "Port";
      bc.is_tacking = false;
    }
    else if (nc.dir_angle>bc.optimal_angle && nc.dir_angle<(180-bc.optimal_angle)){
      bc.point_of_sail = "Reach";
      nc.port_or_starboard = "Port";
      bc.is_tacking = false;
    }
    else if (nc.dir_angle>bc.optimal_angle + 180 && nc.dir_angle<(360-bc.optimal_angle)){
      bc.point_of_sail = "Reach";
      nc.port_or_starboard = "Starboard";
      bc.is_tacking = false;
    }
    else if (nc.dir_angle>(360-bc.optimal_angle)){
      bc.point_of_sail = "Upwind";
      nc.port_or_starboard = "Starboard";
      bc.is_tacking = false;
    }
    else if (nc.dir_angle < (180 + bc.optimal_angle) && nc.dir_angle > 180){
      bc.point_of_sail = "Downwind";
      nc.port_or_starboard = "Starboard";
      bc.is_tacking = false;
    }
    else {
      bc.point_of_sail = "Downwind";
      nc.port_or_starboard = "Port";
      bc.is_tacking = false;
    }
  }
  // Boat is on a starboard tack (wind hitting the sail from the right)
  else {
    if (nc.dir_angle<bc.optimal_angle && nc.dir_angle>0){
      bc.point_of_sail = "Upwind";
      nc.port_or_starboard = "Starboard";
      bc.is_tacking = false;
    }
    else if (nc.dir_angle>bc.optimal_angle && nc.dir_angle<(180-bc.optimal_angle)){
      bc.point_of_sail = "Reach";
      nc.port_or_starboard = "Port";
      bc.is_tacking = false;
    }
    else if (nc.dir_angle>bc.optimal_angle + 180 && nc.dir_angle<(360-bc.optimal_angle)){
      bc.point_of_sail = "Reach";
      nc.port_or_starboard = "Starboard";
      bc.is_tacking = false;
    }
    else if (nc.dir_angle>(360-bc.optimal_angle)){
      bc.point_of_sail = "Upwind";
      nc.port_or_starboard = "Starboard";
      bc.is_tacking = false;
    }
    else if (nc.dir_angle < (180 + bc.optimal_angle) && nc.dir_angle > 180){
      bc.point_of_sail = "Downwind";
      nc.port_or_starboard = "Starboard";
      bc.is_tacking = false;
    }
    else {
      bc.point_of_sail = "Downwind";
      nc.port_or_starboard = "Port";
      bc.is_tacking = false;
    }
  }

  calcIntendedAngle(bc, nc);

  nc.offset = bc.boat_direction - nc.intended_angle;

  float new_tail_angle = nc.wind_direction + nc.offset;
  new_tail_angle = bc.tail_angle - sensorData.boat_direction;

  float new_sail_angle = new_tail_angle + bc.angle_of_attack;
  new_sail_angle = bc.sail_angle - sensorData.boat_direction;

  sensorData.sailAngleBoat = new_sail_angle;
  sensorData.tailAngleBoat = new_tail_angle;

  bc.tail_angle = tailMap(new_sail_angle, new_tail_angle);
  bc.sail_angle = sailMap(new_sail_angle);

  bc.set_sail_angle(bc.sail_angle);
  bc.set_tail_angle(bc.tail_angle);

  printData();
}

// void endurance(Boat_Controller bc, Navigation_Controller nc){
//   //set waypoint array to locations just inside the buoy area
//   if(nc.currentWP = buoyLocations.length){
//     nc.currentWP = 0;
//   }
//   nc.maxDistance = 3;
//   nc.waypoint_array = buoyLocations;
//   nav();
//
// }
 void station_keeping(coord_xy buoyLocations[]){
     if (nc.num_wp == 0) {
       bc.detection_radius = 30;
       if(nc.normal_distance < bc.detection_radius) {
         printData();
         nc.num_wp += 1 ;
        bc.location = sensorData.location;
         nc.normal_distance = xyDist(waypoint_array[nc.num_wp], bc.location);
    //     start_box_time=milTime;
       }
     }
    // else if(time > 4.5 minutes) {
    //   printData();
    //   nc.num_wp += 1;
    //   bc.location = {sensorData.x, sensorData.y};
    //   nc.normal_distance = xyDist(waypoint_array[nc.num_wp], bc.location);
    //   stationKeeping = 0;
    // }
   nav();
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
// bool waypointsSet = false
// void precision_nav(coord_xy buoyLocations[]){
//   if (not waypointsSet) {
//     coord_xy w1 =
//     coord_xy w2 =
//     coord_xy w3 =
//     coord_xy w4 =
//     coord_xy w5 =
//     coord_xy w6 =
//     coord_xy w7 = middlePoint(buoyLocation[0], buoyLocation[1])
//     coord_xy w8 =
//   }
//
//
// }
//
//
// // Ints corresponding to the different events
// int endurance = 1;
// int station_keeping = 2;
// int precision = 3;
// int payload = 4;
// int collison = 5;
// int search = 6;
// int current = 1;
//
// // Offset the buoy locations by hand. Translate to xy etc. buoy locations coord_xy
// // to be 5 meters inside the buoy
// coord_xy buoyLocations[] = {};
// float accSeconds = 0;
//
// void nShort(void){
//   switch (current){
//     case endurance: endurance(buoyLocations)
//     case station_keeping: station_keeping(buoyLocations)
//     case precision:
//     case payload:
//     case collision:
//     case search:
//   }
// }
// */
