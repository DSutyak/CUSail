#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include "sensors.h"
#include "navigation.h"
#include "navigation_helper.h"

void printData(){

  Serial1.print("Y position: "); Serial1.println(sensorData.y,10);
  Serial1.print("X position: "); Serial1.println(sensorData.x,10);

  Serial1.print("latitude: ");  Serial1.println(sensorData.lat,  10);
  Serial1.print("longitude: "); Serial1.println(sensorData.longi,10);


  Serial1.print("Wind w.r.t North: "); Serial1.println(sensorData.wind_dir);
  Serial1.print("Boat direction: "); Serial1.println(sensorData.boat_direction);
  Serial1.print("Distance to Waypoint: "); Serial1.println(xyDist({sensorData.x,sensorData.y},waypoint_array[nc.num_wp]));
  Serial1.print("Angle to Waypoint: "); Serial1.println(angleToTarget({sensorData.x,sensorData.y},waypoint_array[nc.num_wp]));
}

void printWaypointData(){
  Serial1.println("----------NAVIGATION----------");
  Serial1.print("Next Waypoint #");
  Serial1.print(nc.num_wp);
  Serial1.print(": ");
  Serial1.print(waypoint_array[nc.num_wp].x,10);
  Serial1.print(", ");
  Serial1.println(waypoint_array[nc.num_wp].y,10);
}

void printHitWaypointData(){
  Serial1.println("");
  Serial1.println("");
  Serial1.println("");
  Serial1.println("----------");
  Serial1.println("REACHED WAYPOINT!");
  Serial1.println("----------");
  Serial1.println("");
  Serial1.println("");
  Serial1.println("");
}

void printSailTailSet(){
  Serial1.print("Sail angle: ");   Serial1.println(bc.sail_angle);
  Serial1.print("Tail angle: ");   Serial1.println(bc.tail_angle);
}
