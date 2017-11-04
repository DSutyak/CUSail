#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include "sensors.h"
#include "navigation.h"

void printData(){
  Serial1.print("X Coord: "); Serial1.println(sensorData.y,10);
  Serial1.print("Y Coord: "); Serial1.println(sensorData.x,10);
  Serial1.print("Wind w.r.t North: "); Serial1.println(sensorData.windDir);
  Serial1.print("Boat direction: "); Serial1.println(sensorData.boatDir);
}

void printWaypointData(){
  Serial1.println("----------NAVIGATION----------");
  Serial1.print("Next Waypoint #");
  Serial1.print(wpNum);
  Serial1.print(": ");
  Serial1.print(wayPoints[wpNum].y,10);
  Serial1.print(", ");
  Serial1.println(wayPoints[wpNum].x,10);
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
  Serial1.print("Sail angle: ");   Serial1.println(sailAngle);
  Serial1.print("Tail angle: ");   Serial1.println(tailAngle);
}
