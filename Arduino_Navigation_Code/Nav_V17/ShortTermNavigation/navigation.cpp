#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include "print.h"
#include "sensors.h"
#include "navigation.h"
#include "navigation_helper.h"

use namespace std

/*
An object of class Boat_Controller represents the boat navigating in the water.
It handles all variables specicic to the state of the boat itself.
*/

public class Boat_Controller {
  float sail_angle;
  float tail_angle;
  float boat_direction;
  coord_xy location;
  Servo tailServo;
  Servo sailServo;
  float detection_radius;
  float port_boundary;
  float starboard_boundary;

  float angle_of_attack = 10;
  float static optimal_angle = 60;

  /*
  Constructor for a boat. Sets up the servos, and establishes intial values
  for each boat vaiable.

  Arguments:
    detection_radius is a float that represents how close we have to
    be to a waypoint to mark it as 'hit'. Precondition: detection_radius must
    be less than the upper and lower tacking bounds.
  */
  Boat_Controller(float d){
    sail_angle = set_sail_angle(0.00);
    tail_angle = set_tail_angle(0.00);
    boat_direction = sensorData.boatDir;
    location = currentPosition = {sensorData.x, sensorData.y};
    tailServo.attach(tailServoPin);
    sailServo.attach(sailServoPin);
    detection_radius = d;
    initSensors();
  }

  //Sets the angle of the main sail
  void set_sail_angle (float angle){
    sailServo.write(angle);
  }

  //Sets the angle of the tail sail
  void set_tail_angle (float angle){
    tailServo.write(angle);
  }

}

/*
An object of class Navigation_Controller represents the abstract
(not directly related to the boat) variables and operations performed on them to
navigate a body of water.

port and starboard are floats that represent
how far (in meters) to port and starboard the boat is allowed to
go before tacking.
*/
  public class Navigation_Controller{
    coord_xy waypoint_array[];
    float angleToWaypoint;
    float normalDistance;
    string pointOfSail;
    float intendedAngle;
    string portOrStarboard;
    float maxDistance;
    int numWP;
    int currentWP;
    float dirAngle;
    float offset;
    float wind_direction;
    float port_boundary;
    float starboard_boundary;
    /*
    Constructor for a Navigation Controller.
    Sets up the waypoints, and establishes
    default values for each navigation variable.

    Arguments:
      max is a float that represents how far we can get from the origin before
      the origin must be reset.

      num is an int that represents the number of coord_xy waypoints in the
      array waypoints argument.
    */
    Navigation_Controller(float max, int num, coord_xy array waypoints,
    float port, float starboard){
      waypoint_array = waypoints;
      angleToWaypoint = 0.0;
      normalDistance = 0.0;
      pointOfSail = "";
      intendedAngle = 0.0;
      portOrStarboard = ""
      maxDistance = max;
      numWP = num;
      dirAngle = 0.0;
      offset = 0.0;
      wind_direction = 0.0;
      port_boundary = port;
      starboard_boundary = starboard;
    }
  }

  void main(float windDir, float boatDir) {
    windDir = sensorData.windDir;
    boatDir = sensorData.boatDir;
    calcIntendedAngle(PointofSail, portOrStarboard, windDir, angleToWaypoint);
    if (detection_radius >= normalDistance) {
      (if currentWP != numWP) {
        //to-do: update location
        currentWP++; //to-do: syntax for next el in list
        if ((boatDir - windDir) % 360 < 180) {
            portOrStarboard = "Port";
        }
        else {
          portOrStarboard = "Starboard";
        }
      }
    }
    //to-do: bounds
    //to-do: pass into navigation_helper
    //to-do: update sail and tail angle
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
  void calcIntendedAngle(string PointofSail, string portOrStarboard, float windDir, float angleToWaypoint) {
    if (PointofSail != "Upwind" && PointofSail != "Reach" && PointofSail != "Downwind" ) {
      Serial1.print("Invalid argument sent to nav");
    }
    else if (PointofSail == "Upwind") {
      if (portOrStarboard == "Port") {
        Serial1.print("UPWIND PORT");
        intendedAngle = windDir - optimal_angle;
        angle_of_attack = -15;
      }
      else {
        Serial1.print("UPWIND STARBOARD");
        intended_angle= windDir + optimal_angle;
        angle_of_attack = 15;
      }
    }
    else if (PointofSail == "Reach") {
      if (portOrStarboard == "Port") {
        Serial1.print("REACH PORT");
        intended_angle = angleToWaypoint;
        angle_of_attack = -15;
      }
      else {
        Serial1.print("REACH STARBOARD");
        intended_angle = angleToWaypoint;
        angle_of_attack = 15;
      }
    }
    else{
       if(portOrStarboard == "Port") {
         Serial1.print("DOWNWIND PORT");
         intended_angle = windDir + 180 + optimal_angle;
         intended_angle_of_attack = -15;
       }
       else{
         Serial1.print("DOWNWIND STARBOARD");
         intended_angle = windDir + 180 - optimal_angle;
         intended_angle_of_attack = 15;
       }
  }

  /*
  Helper function used to determine whether the boat is aboveBounds or belowBounds.

  Arguments:
    width is a float that sets the maximum allowable width for boat to sail within

    point1 and point2 are coordinates in the xy plane
  */
  string calcBounds(float width=10, coord_xy point1, coord_xy point2, string pointOfSail) {
    float slope = xySlope(point1, point2);
    float intercept = point1.y - slope * point1.x;
    float distance = -1*(slope * point1.x - point1.y + intercept)/sqrtf(intercept*intercept+1);
    if (distance > width) {
      return "portBounds";
    }
    else {
      return "starboardBounds";
    }
  }
