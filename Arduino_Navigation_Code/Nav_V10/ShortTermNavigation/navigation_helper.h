/*Returns angle (with respect to North) between two global coordinates.*/
float angleToTarget(float lat1, float long1, float lat2, float long2);

/*Returns great circle distance (in meters) between two global coordinates.*/
double havDist(coord_t  first, coord_t second);

/*converts an angle to a 0-360 range*/
float convertto360(float angle);

/*------------------------------------------*/
/*----------Tailvane-angle setters----------*/
/*------------------------------------------*/

// orientation is 1 for right and 0 for left
// right is negative offset, left is positive
// never go upright when facing left
// facing right, angle is above in the sector: w-offset
//   w-(|w+opttop - boatdir|)
float upRight(float b, float w);

float rightTarget(float b, float w);

float leftTarget(float b, float w);

// facing left, angle is above in the sector: w+offset
//   w+(|w-opttop-boatdir|)
float upLeft(float b, float w);

// facing left, angle is below in the sector: w+offset
//   w+(|w+180-optboat-boatdir|)
float downLeft(float b, float w);
// facing right, angle is below in the sector: w-offset
//   w-(|w+180+optbot-boatdir|)
float downRight(float b, float w);

/*------------------------------------------*/

// float* buoyRounding(coord_t b, coord_t wp1, coord_t wp2, float initial_anglewaypoint, float normr);
