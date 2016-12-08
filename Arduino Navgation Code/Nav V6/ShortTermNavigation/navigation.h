/*-----------------------------------------------------------------
 CU Sail
 Cornell University Autonomous Sailboat Team

 Navigation
 Allows determination of the appropriate Sail and Tail angle and setting of the Servos

 Code has been tested and run on an Arduino Due

 Sail Servo: HS-785HB by Hitec
 Tail Servo: HS-5646WP by Hitec

 Last Updated: 10/20/2016 by Arjan Singh
 
 Authors: Alex Pomerenk, Alec Dean, Arjan Singh, Caleb Koch, Stephanie Hogan
 
 Past Contributors: Eric T. J. Jung, Brian Gross, Varun Shanbhag, David Brown
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

