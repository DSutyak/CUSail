/*-----------------------------------------------------------------
 CU Sail
 Cornell University Autonomous Sailboat Team

 Navigation
 Allows determination of the appropriate Sail and Tail angle and setting of the Servos

 Code has been tested and run on an Arduino Due

 Sail Servo: HS-785HB by Hitec
 Tail Servo: HS-5646WP by Hitec
--------------------------------------------------------------------*/

/*----------Type Definitions----------*/
typedef struct coordinate {
  double latitude; // float latitude
  double longitude; // float longitudes
} coord_t;

typedef struct coord_xy {
  double x; // float x coord
  double y; // float y coord
} coord_xy;

/*----------Predefined Variables----------*/
#define maxPossibleWaypoints 100
#define tailServoPin 8
#define sailServoPin 9
//Optimal angle to go at if we cannot go directly to the waypoint
//Puts us on a tack or jibe
//Different values for top and bottom of polar plot
// #define optPolarTop 60//20
// #define optPolarBot 60//40
// #define angleOfAttack 10
// #define detectionRadius 15

/*----------Global Variables-----------*/
extern coord_xy wayPoints[maxPossibleWaypoints]; //the array containing the waypoints with type coord_t


/*----------Functions----------*/
void initializer(void);

void calcIntendedAngle(Boat_Controller bc, Navigation_Controller nc);
/*Determines whether boat is above upper boundary
*/
bool aboveBounds(float upperWidth, coord_xy location, coord_xy nextwp, string pointOfSail);
/*Determines whether boat is below lower boundaryd
*/
bool belowBounds(float lowerWidth, coord_xy location, coord_xy nextwp, string pointOfSail);

/*Sets sail and tail angle given information from nShort */
void nav(int quadrant, bool rightLeft);
