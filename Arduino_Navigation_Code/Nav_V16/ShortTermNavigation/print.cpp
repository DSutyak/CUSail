  #include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include "sensors.h"
#include "navigation.h"
#include "navigation_helper.h"

void printData(){
  
  String outString = "";
  outString = ("Y position: ") + (String)(sensorData.y);
  outString += ("X position: ") + (String)(sensorData.x);
  byte out[sizeof(outString)];
  outString.getBytes(out, sizeof(outString));
  Tx16Request tx = Tx16Request(0x1874, out, sizeof(outString));
  xbee.send(tx);
  
  outString = ("latitude: ") +  (String)(sensorData.lat);
  outString += ("longitude: ") + (String)(sensorData.longi);
  byte out2[sizeof(outString)];
  outString.getBytes(out2, sizeof(outString));
  tx = Tx16Request(0x1874, out2, sizeof(outString));
  xbee.send(tx);

  outString = ("Wind w.r.t North: ") + (String)(sensorData.windDir);
  outString += ("Boat direction: ") + (String)(sensorData.boatDir);
  byte out3[sizeof(outString)];
  outString.getBytes(out3, sizeof(outString));
  tx = Tx16Request(0x1874, out3, sizeof(outString));
  xbee.send(tx);
  
  outString = ("Distance to Waypoint: ") +  (String)(xyDist({sensorData.x,sensorData.y},wayPoints[wpNum]));
  outString += ("Angle to Waypoint: ") + (String)(angleToTarget({sensorData.x,sensorData.y},wayPoints[wpNum]));
  byte out4[sizeof(outString)];
  outString.getBytes(out4, sizeof(outString));
  tx = Tx16Request(0x1874, out4, sizeof(outString));
  xbee.send(tx);
}

void printWaypointData(){
  Serial1.println("----------NAVIGATION----------");
  Serial1.print("Next Waypoint #");
  Serial1.print(wpNum);
  Serial1.print(": ");
  Serial1.print(wayPoints[wpNum].x,10);
  Serial1.print(", ");
  Serial1.println(wayPoints[wpNum].y,10);
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
