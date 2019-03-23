#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include "sensors.h"
#include "navigation.h"

void printData(){

  Serial1.print("Y position: "); Serial1.println(sensorData.y,10);
  Serial1.print("X position: "); Serial1.println(sensorData.x,10);

  Serial1.print("latitude: ");  Serial1.println(sensorData.lat,  10);
  Serial1.print("longitude: "); Serial1.println(sensorData.longi,10);


  Serial1.print("Wind w.r.t North: "); Serial1.println(sensorData.windDir);
  Serial1.print("Boat direction: "); Serial1.println(sensorData.boatDir);
  
  //only really usefull for quad tests, second one is a bit flukey
  Serial1.print("Distance to Waypoint: ")    ;Serial1.println(xyDist({sensorData.x,sensorData.y},wayPoints[wpNum]));
  Serial1.print("Angle to Waypoint: ");Serial1.println(angleToTarget({sensorData.x,sensorData.y},wayPoints[wpNum]));

  Serial1.println("----------NAVIGATION----------");
  Serial1.print("Lat-Long Coordinate: ("); Serial1.print(sensorData.lat,10); Serial1.print(", "); 
  Serial1.print(sensorData.longi,10); Serial1.println(")"); Serial1.print("\n");
  Serial1.print("Wind w.r.t North: "); Serial1.println(sensorData.windDir); Serial1.print("\n");
  Serial1.print("Boat direction: "); Serial1.print(sensorData.boatDir); Serial1.print("\n");
}

void printWaypointData(){
  Serial1.print("Next Waypoint: #");
  Serial1.print(wpNum);
  Serial1.print(", (");
  Serial1.print(wayPoints[wpNum].x,10);
  Serial1.print(", ");
  Serial1.print(wayPoints[wpNum].y,10);
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
