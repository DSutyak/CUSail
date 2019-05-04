
#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include "sensors.h"
#include "navigation.h"
#include "navigation_helper.h"

void printData(){
  Serial1.println("----------NAVIGATION----------");
  
  Serial1.print("Y position: "); Serial1.println(bc.location.y,10);
  Serial1.print("X position: "); Serial1.println(bc.location.x,10);

  Serial1.print("latitude: ");  Serial1.println(sensorData.lat,  10);
  Serial1.print("longitude: "); Serial1.println(sensorData.longi,10);


  Serial1.print("Wind w.r.t North: "); Serial1.println(nc.wind_direction);
  Serial1.print("Boat direction: "); Serial1.println(bc.boat_direction);
  Serial1.print("Distance to Waypoint: "); Serial1.println(nc.normal_distance);
  Serial1.print("Angle to Waypoint: "); Serial1.println(nc.angle_to_waypoint);
  Serial1.print("Roll: "); Serial1.println(sensorData.roll);
  Serial1.print("Pitch: "); Serial1.println(sensorData.pitch);

  Serial1.print("Next Waypoint #:");
  Serial1.println(nc.current_wp);
  Serial1.print("Next Waypoint X: ");
  Serial1.println(waypoint_array[nc.current_wp].x,10);
  Serial1.print("Next Waypoint Y: ");
  Serial1.println(waypoint_array[nc.current_wp].y,10);

  Serial1.print("Sail angle: ");   Serial1.println(bc.sail_angle);
  Serial1.print("Tail angle: ");   Serial1.println(bc.tail_angle);
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
