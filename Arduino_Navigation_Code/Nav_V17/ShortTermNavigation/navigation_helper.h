/*-----------------------------------------------------------------
 CU Sail
 Cornell University Autonomous Sailboat Team

 Navigation_helper
 Provides helper functions for Navigation
--------------------------------------------------------------------*/
#include "navigation.h"

extern coord_xy origin;
/*Sets the origin of the XY plane to a starting point*/
void setOrigin(coord_t startPoint);

/*Converts a lat-long point to be a Cartesian point with respect to the origin*/
coord_xy xyPoint(coord_t latlong);

/*Finds the distance between two Cartesian coordiantes*/
double xyDist(coord_xy point1, coord_xy point2);

/*Changes the sail and tail on the detection of an object*/
void avoidObject(void);

/*Returns angle (with respect to North) between two global coordinates.*/
float angleToTarget(coord_xy coord1, coord_xy coord2);

/*Returns great circle distance (in meters) between two global coordinates.*/
double havDist(coord_t first, coord_t second);

/*converts an angle to a 0-360 range*/
float convertto360(float angle);

/*Calculates slope between two xy coordinate points*/
float xySlope(coord_xy point1, coord_xy point2);

/*Finds coordinate directly between two xy coordinate points*/
coord_xy middlePoint(coord_xy point1, coord_xy point2);
