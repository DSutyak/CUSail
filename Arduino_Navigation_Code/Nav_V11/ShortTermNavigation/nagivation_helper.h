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
  double longitude; // float longitude
} coord_t;

/*----------Predefined Variables----------*/
#define maxPossibleWaypoints 100
#define tailServoPin 8
#define sailServoPin 9
//Optimal angle to go at if we cannot go directly to the waypoint
//Puts us on a tack or jibe
//Different values for top and bottom of polar plot
#define optPolarTop 45
#define optPolarBot 40
#define angleOfAttack 15
#define detectionRadius 10

/*----------Global Variables-----------*/
extern float sailAngle;
extern float tailAngle;
extern int wpNum; //the current waypoint's number in the wayPoints array
extern coord_t wayPoints[maxPossibleWaypoints]; //the array containing the waypoints with type coord_t

/*----------Functions----------*/
/*Servos setup
* "Attaches" sail servo and tail servo.*/
void initServos(void);

/*Navigation algorithm setup.
* Sets current waypoint number and total number of waypoints to 0*/
void initNavigation(void);

/*Sets waypoints for navigation
* by creating the wayPoints array*/
void setWaypoints(void);

/*Short Term Navigation Algorithm
* Uses sensorData.windDir, sensorData.boatDir to set sailAngle and tailAngle. 
* sailAngle and tailAngle are set in terms of servo command numbers, but are first
* calculated in terms of angle w.r.t the boat direction*/
void nShort(void);

/*Sets servos to the sailAngle and tailAngle*/
void nServos(void);




/*Returns angle (with respect to North) between two global coordinates.*/
float angleToTarget(float lat1, float long1, float lat2, float long2);

/*Changes the intended_angle_of_attack and intended_angle of the boat
given the sector the boat is in and the location of the next waypoint.
And then use the offset (a positive or negative angle that shows the 
amount that we are facing away from the intended angle) and wind direction
to set the sail then offset the sail to the left or right WRT the tail
to give it an angle of attack */
void set_boat_angle(float intended_angle, float intended_angle_of_attack);

void waypoint_reached();

void obstacle_avoidance();

/*Returns great circle distance (in meters) between two global coordinates.*/
double havDist(coord_t  first, coord_t second);

// converts an angle to a 0-360 range
float convert_to_360(float angle);

