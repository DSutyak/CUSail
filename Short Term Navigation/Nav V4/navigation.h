// -----------------------------------------------------------------
// Cornell Autonomous Sail Boat Team
// Author: Eric T. J. Jung, Brian Gross, Varun Shanbhag, David Brown
// Stephanie Hogan, Qiukai Lin
//
// Implements a library of functions for gathering data from sensors
// -----------------------------------------------------------------

#ifndef __NAVIGATION_H__
#define __NAVIGATION_H__


/*------------------------------ Structures ------------------------------*/
typedef struct coordinate {
  double latitude; // float latitude;
  double longitude; // float longitude;
} coord_t;


/*----------------------- Predefined Variables -----------------------*/
#define MAX_WAYPOINTS     100
#define wpErr               1 //4 // 1 //.5
//#define wpErr               .0005
#define tailMid           90
#define tailMax           15
#define tailServoPin       8 //45
#define sailMid           90
#define sailMax           15
#define sailServoPin       9 //43        // servo 1
#define Pc                 5
//#define delalpha          1
#define controlP           1   // P control parameter for PD controller
#define controlD           .5  // P control parameter for PD controller


/*--------------------------- Global Variables ---------------------------*/
extern float tailAngle;
extern int wpNum;

extern coord_t wayPoints[MAX_WAYPOINTS];

/*------------------------------ Functions ------------------------------*/
// initializes servos
void initServos(void);

// initializes navigation algorithm
void initNavigation(void);

// sets waypoints
//void setWaypoints(coord_t wPoint[]);
void setWaypoints(void);

// short term navigation algorithm
void nShort(void);

// adjusts servos
void nServos(void);

// calibrates servos with serial monitor input
void calSail(void);
void caltail(void);

// calculate distance in meters given two lat/long points
float toMeters(float lat1, float long1, float lat2, float long2);

#endif /* __NAVIGATION_H__ */
