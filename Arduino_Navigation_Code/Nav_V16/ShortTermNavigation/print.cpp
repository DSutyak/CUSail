#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include "sensors.h"
#include "navigation.h"

void printData(){
  Serial1.println("----------NAVIGATION----------");
  Serial1.print("Lat-Long Coordinate: ("); Serial1.print(sensorData.lati,10); Serial1.print(", "); 
  Serial1.print(sensorData.longi,10); Serial1.println(")"); Serial1.print("\n");
  Serial1.print("Wind w.r.t North: "); Serial1.println(sensorData.windDir); Serial1.print("\n");
  Serial1.print("Boat direction: "); Serial1.print(sensorData.boatDir); Serial1.print("\n");
}

void printWaypointData(){
  Serial1.print("Next Waypoint: #");
  Serial1.print(wpNum);
  Serial1.print(", (");
  Serial1.print(wayPoints[wpNum].latitude,10);
  Serial1.print(", ");
  Serial1.print(wayPoints[wpNum].longitude,10);
  Serial1.print(")"); Serial1.print("\n");
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
  Serial1.print("Sail angle: ");   Serial1.print(sailAngle); Serial1.print("\n");
  Serial1.print("Tail angle: ");   Serial1.print(tailAngle); Serial1.print("\n");
}