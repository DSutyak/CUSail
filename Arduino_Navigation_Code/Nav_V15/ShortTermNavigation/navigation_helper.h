/*-----------------------------------------------------------------
 CU Sail
 Cornell University Autonomous Sailboat Team

 Navigation_helper
 Provides helper functions for Navigation
--------------------------------------------------------------------*/

/*Changes the sail and tail on the detection of an object*/
void avoidObject(void);

/*Returns angle (with respect to North) between two global coordinates.*/
float angleToTarget(float lat1, float long1, float lat2, float long2);

/*Returns great circle distance (in meters) between two global coordinates.*/
double havDist(coord_t  first, coord_t second);

/*converts an angle to a 0-360 range*/
float convertto360(float angle);
