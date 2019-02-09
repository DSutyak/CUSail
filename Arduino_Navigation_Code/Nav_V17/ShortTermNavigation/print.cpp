#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include "sensors.h"
#include "navigation.h"
#include "navigation.cpp"
#include "navigation_helper.h"

void printData(){

  Serial1.print("Y position: "); Serial1.println(sensorData.y,10);
  Serial1.print("X position: "); Serial1.println(sensorData.x,10);

  Serial1.print("latitude: ");  Serial1.println(sensorData.lat,  10);
  Serial1.print("longitude: "); Serial1.println(sensorData.longi,10);


  Serial1.print("Wind w.r.t North: "); Serial1.println(sensorData.windDir);
  Serial1.print("Boat direction: "); Serial1.println(sensorData.boatDir);
  Serial1.print("Distance to Waypoint: "); Serial1.println(xyDist({sensorData.x,sensorData.y},wayPoints[numWP]));
  Serial1.print("Angle to Waypoint: "); Serial1.println(angleToTarget({sensorData.x,sensorData.y},wayPoints[numWP]));
}

void printWaypointData(){
  Serial1.println("----------NAVIGATION----------");
  Serial1.print("Next Waypoint #");
  Serial1.print(numWP);
  Serial1.print(": ");
  Serial1.print(wayPoints[numWP].x,10);
  Serial1.print(", ");
  Serial1.println(wayPoints[numWP].y,10);
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
