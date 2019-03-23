/*-----------------------------------------------------------------
 CU Sail
 Cornell University Autonomous Sailboat Team

 Navigation
 Allows determination of the appropriate Sail and Tail angle and setting of the Servos

 Code has been tested and run on an Arduino Due

 Sail Servo: HS-785HB by Hitec
 Tail Servo: HS-5646WP by Hitec
--------------------------------------------------------------------*/
#include "Servo.h"
#include <String.h>
#include <WString.h>
#include <Servo.h>
#include <Arduino.h>
#include "coordinates.cpp"

/*----------Type Definitions----------*/
#ifndef coordinate_h
#define coordinate_h

typedef struct coord_t {
  double latitude; // float latitude
  double longitude; // float longitudes
}coord_t;
#endif

#ifndef coord_xy_h
#define coord_xy_h
typedef struct _coord_xy {
  double x; // float x coord
  double y; // float y coord
} coord_xy;
#endif

/*----------Type Definitions----------*/
#ifndef Boat_Controller_h
#define Boat_Controller_h

class Boat_Controller{
public:
  float sail_angle;
  float tail_angle;
  float boat_direction;
  _coord_xy location;
  Servo tailServo;
  Servo sailServo;
  float detection_radius;
  bool is_tacking;
  String point_of_sail;
  float angle_of_attack = 10;
  constexpr float static optimal_angle = 60;

  //Sets the angle of the main sail
  void set_sail_angle (float angle){
    sailServo.write(angle);
  }

  //Sets the angle of the tail sail
  void set_tail_angle (float angle) {
    tailServo.write(angle);
  }


  /*
  Constructor for a boat. Sets up the servos, and establishes intial values
  for each boat variable.

  Arguments:
    detection_radius is a float that represents how close we have to
    be to a waypoint to mark it as 'hit'. Precondition: detection_radius must
    be less than the upper and lower tacking bounds.
  */
  Boat_Controller(){}
  void boat_init (float d, int sailServoPin, int tailServoPin) {
    set_sail_angle(0.00);
    set_tail_angle(0.00);
    boat_direction = sensorData.boat_direction;
    tailServo.attach(tailServoPin);
    sailServo.attach(sailServoPin);
    detection_radius = d;
    is_tacking = false;
    point_of_sail = "";
  }  //Sets the angle of the main sail
};
#endif
/*
An object of class Navigation_Controller represents the abstract
(not directly related to the boat) variables and operations performed on them to
navigate a body of water.
*/
#ifndef Navigation_Controller_h
#define Navigation_Controller_h
class Navigation_Controller {
public:
  float angle_to_waypoint;
  float normal_distance;
  float intended_angle;
  String port_or_starboard;
  float max_distance;
  int num_wp;
  int current_wp;
  float dir_angle;
  float offset;
  float wind_direction;
  float upper_width; //boundary on port side
  float lower_width; //boundary on starboard side

/*
An object of class Navigation_Controller represents the abstract
(not directly related to the boat) variables and operations performed on them to
navigate a body of water.
*/

  /*
  Constructor for a Navigation Controller.
  Sets up the waypoints, and establishes
  default values for each navigation variable.

  Arguments:
    max is a float that represents how far we can get from the origin before
    the origin must be reset.

    num is an int that represents the number of coord_xy waypoints in the
    array waypoints argument.

    port and starboard are floats that represent
    how far (in meters) to port and starboard the boat is allowed to
    go before tacking.
  */
  Navigation_Controller(){}
  void nav_init (float limit, int num, coord_xy waypoints[], float port, float starboard){
    int count = 0;
//    while (count < num){
//      waypoint_array[count] = waypoints[count];
//      count++;
//    }
    angle_to_waypoint = 0.0;
    normal_distance = 0.0;
    intended_angle = 0.0;
    port_or_starboard = "";
    max_distance = limit;
    num_wp = num;
    current_wp = 0;
    dir_angle = 0.0;
    offset = 0.0;
    wind_direction = 0.0;
    upper_width = 10;
    lower_width = 10;
  }
};

#endif

/*----------Predefined Variables----------*/
#define maxPossibleWaypoints 100
#define tailServoPin 8
#define sailServoPin 9
//Optimal angle to go at if we cannot go directly to the waypoint
//Puts us on a tack or jibe
//Different values for top and bottom of polar plot
#define optPolarTop 60//20
#define optPolarBot 60//40
#define angleOfAttack 10
#define detectionRadius 15

/*----------Global Variables-----------*/
extern _coord_xy waypoint_array[maxPossibleWaypoints]; //the array containing the waypoints with type coord_t
extern Boat_Controller bc;
extern Navigation_Controller nc;

/*----------Functions----------*/
void initializer(void);

void calcIntendedAngle(Boat_Controller bc, Navigation_Controller nc);
/*Determines whether boat is above upper boundary
*/
bool aboveBounds(float upperWidth, _coord_xy location, _coord_xy nextwp, String pointOfSail);
/*Determines whether boat is below lower boundaryd
*/
bool belowBounds(float lowerWidth, _coord_xy location, _coord_xy nextwp, String pointOfSail);

/*Sets sail and tail angle given information from nShort */
void nav();
