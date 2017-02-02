#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include "sensors.h"
#include "navigation.h"

/*----------------------------*/
/*----------Printers----------*/
/*----------------------------*/

void printLocationData(){
  Serial1.print("Latitude: "); Serial1.println(sensorData.lati);
  Serial1.print("Longitude: "); Serial1.println(sensorData.longi);
}

void printWaypointData(){
  Serial1.println("----------NAVIGATION----------");

  Serial1.print("Next Waypoint #");
  Serial1.print(wpNum);
  Serial1.print(": ");
  Serial1.print(wayPoints[wpNum].latitude);
  Serial1.print(", ");
  Serial1.println(wayPoints[wpNum].longitude);
//  Serial1.print("Distance to waypoint: ");Serial1.println(normr);
//  Serial1.print("Detection Radius: ");Serial1.println(detectionradius);
}

void printHitWaypointData(){
  Serial1.println("");
  Serial1.println("");
  Serial1.println("");
  Serial1.println("----------");
  Serial1.println("REACHED WAYPOINT");
  Serial1.println("----------");
  Serial1.println("");
  Serial1.println("");
  Serial1.println("");

  Serial1.println("Reached waypoint #");
  Serial1.print(wpNum);
  Serial1.print(": ");
  Serial1.print(wayPoints[wpNum].latitude);
  Serial1.print(", ");
  Serial1.println(wayPoints[wpNum].longitude);
}

void printSailTailSet(){
  Serial1.print("Sail angle (0 to 360) w.r.t North: ");   Serial1.println(sailAngle);
  Serial1.print("Tail angle (0 to 360) w.r.t North: ");   Serial1.println(tailAngle);
  //Print boat and wind direction to make sure data is consistent at this point
  Serial1.print("sensorData.boatDir: ");   Serial1.println(sensorData.boatDir);
  Serial1.print("sensorData.windDir: ");   Serial1.println(sensorData.windDir);
}

void printServoSailTail(){
  Serial1.print("Sail angle (Servo Command): ");   Serial1.println(sailAngle);
  Serial1.print("Tail angle (Servo Command): ");   Serial1.println(tailAngle);
  Serial1.println("");
  Serial1.println("--------------------");
  Serial1.println("");
}
