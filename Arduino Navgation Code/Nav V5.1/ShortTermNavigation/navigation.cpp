#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include "sensors.h"
#include "navigation.h"

//Define Servo types for Sail and Tail
Servo tailServo;
Servo sailServo;


/*----------Polar Plot of Sailvane----------
* Gives expected speed of boat at a given heading with respect to the wind. 
* The element number of array represents angle w.r.t wind.
* 0 degrees heading represents sailing directly into the wind. Angle increases clockwise*/
float polarPlot [361] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0.1712,0.17325,0.17742,0.18336,0.19064,0.19882,0.20757,0.21661,
                        0.22575,0.23487,0.24389,0.25275,0.26143,0.26991,0.27818,0.28623,0.29407,0.3017,0.30912,0.31634,
                        0.32337,0.33021,0.33687,0.34335,0.34966,0.35581,0.36179,0.36761,0.37329,0.37882,0.3842,0.38944,
                        0.39455,0.39952,0.40437,0.40909,0.41368,0.41815,0.42251,0.42675,0.43087,0.43488,0.43878,0.44257,
                        0.44626,0.44984,0.45332,0.4567,0.45998,0.46315,0.46624,0.46922,0.47211,0.47491,0.47761,0.48023,
                        0.48275,0.48518,0.48753,0.48979,0.49196,0.49405,0.49605,0.49797,0.4998,0.50155,0.50322,0.50481,
                        0.50632,0.50774,0.50909,0.51036,0.51155,0.51267,0.5137,0.51466,0.51555,0.51636,0.51709,0.51775,
                        0.51833,0.51884,0.51927,0.51964,0.51992,0.52014,0.52028,0.52035,0.52035,0.52028,0.52013,0.51991,
                        0.51962,0.51926,0.51883,0.51833,0.51775,0.51711,0.51639,0.5156,0.51474,0.51381,0.51281,0.51173,
                        0.51059,0.50937,0.50808,0.50672,0.50529,0.50378,0.5022,0.50055,0.49882,0.49702,0.49514,0.49319,
                        0.49117,0.48907,0.48689,0.48463,0.4823,0.47989,0.4774,0.47483,0.47218,0.46945,0.46663,0.46373,
                        0.46075,0.46373,0.46663,0.46945,0.47218,0.47483,0.4774,0.47989,0.4823,0.48463,0.48689,0.48907,
                        0.49117,0.49319,0.49514,0.49702,0.49882,0.50055,0.5022,0.50378,0.50529,0.50672,0.50808,0.50937,
                        0.51059,0.51173,0.51281,0.51381,0.51474,0.5156,0.51639,0.51711,0.51775,0.51833,0.51883,0.51926,
                        0.51962,0.51991,0.52013,0.52028,0.52035,0.52035,0.52028,0.52014,0.51992,0.51964,0.51927,0.51884,
                        0.51833,0.51775,0.51709,0.51636,0.51555,0.51466,0.5137,0.51267,0.51155,0.51036,0.50909,0.50774,
                        0.50632,0.50481,0.50322,0.50155,0.4998,0.49797,0.49605,0.49405,0.49196,0.48979,0.48753,0.48518,
                        0.48275,0.48023,0.47761,0.47491,0.47211,0.46922,0.46624,0.46315,0.45998,0.4567,0.45332,0.44984,
                        0.44626,0.44257,0.43878,0.43488,0.43087,0.42675,0.42251,0.41815,0.41368,0.40909,0.40437,0.39952,
                        0.39455,0.38944,0.3842,0.37882,0.37329,0.36761,0.36179,0.35581,0.34966,0.34335,0.33687,0.33021,
                        0.32337,0.31634,0.30912,0.3017,0.29407,0.28623,0.27818,0.26991,0.26143,0.25275,0.24389,0.23487,
                        0.22575,0.21661,0.20757,0.19882,0.19064,0.18336,0.17742,0.17325,0.1712,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

/*----------Navigation Variables----------*/
int wpNum; //the current waypoint's number in the wayPoints array
int numWP; //total number of waypoints on current course
float detectionradius=3; //how far away the boat marks a waypoint "reached"
coord_t wayPoints[maxPossibleWaypoints]; //the array containing the waypoints
float normr; //normal distance to the waypoint
float r[2]; //r[0]: Longitude difference b/w current position and waypoint, 
            //r[1]: Lattitude difference b/w current position and waypoint
float w[2]; //w[0]: Cosine of the wind direction w.r.t North,
            //w[1]: Sine of the wind direction w.r.t North
float sailAngle;
float tailAngle;
float angleofattack;
float optpolartop;
float optpolarbot;

/*----------Stored Coordinates----------*/
//Coordinates in and around the Engineering Quad, Cornell university
coord_t olin = {42.445457, -76.484322}; //Olin Hall
coord_t hollister = {42.444259, -76.484435}; //Hollister Hall
coord_t outsideDuffield = {42.444254, -76.482660}; //Outside West entrance to Duffield Hall, atop the stairs
coord_t outsideThurston = {42.444228, -76.483666}; //In front of Thurston Hall
coord_t engQuadX = {42.444591, -76.483498}; //Center of Engineering Quad
coord_t bottomStairs = {42.444240, -76.483258}; //Bottom of stairs, outside West entrance to Duffield Hall
coord_t engQuadRight = {42.4444792, -76.483244}; //Bottom of stairs, to the left of North entrance to Duffield Hall

//Coordinates in and around the Cornell Sailing Center
coord_t lakeOut = {42.468792,-76.5044053}; //Out in the lake, to the left of the Cornell Sailing Center
coord_t lakeOut2 = {42.469065,-76.506674}; //Out in the lake, to the right of the Cornell Sailing Center
coord_t shore = {42.469033,-76.5040623}; //Far end of the docks, to the left of the Cornell Sailing Center

/*Servo setup
* "Attaches" servos to defined pins*/
void initServos(void) {
  tailServo.attach(tailServoPin);
  sailServo.attach(sailServoPin);
}

/*Navigation algorithm setup.
* Sets curretn waypoint number and total number of waypoints to 0*/
void initNavigation(void) {
  wpNum = 0;
  numWP = 0;
}

/*Sets waypoints for navigation
* by creating the wayPoints array*/
void setWaypoints(void) {

  //Make the waypoint array
  numWP = 1;
  wpNum = 0;

  //Set way points to desired coordinates.
  //Assignmment must be of the type coord_t.
  wayPoints[0] = lakeOut2;
  
  //Serial prints to Serial Monitor
  Serial.println("First coordinate: ");
  Serial.print("latitude: "); Serial.println(wayPoints[0].longitude,6);
  Serial.print("longitude: "); Serial.println(wayPoints[0].latitude,6);

  
  //Serial1 print through the XBees
  Serial1.println("First coordinate: ");
  Serial1.print("latitude: "); Serial.println(wayPoints[0].longitude,6);
  Serial1.print("longitude: "); Serial.println(wayPoints[0].latitude,6);
}

void tack(void){
  //Luey psuedo code

}


/*Returns angle (with respect to North) between two global coordinates.
* Coordinates must be given in latitude and longitude */
float angleToTarget(float lat1, float long1, float lat2, float long2){
  lat1=lat1 * M_PI/180;
  lat2=lat2 * M_PI/180;
  float dLong=(long2-long1) * M_PI/180;
  float y = sin(dLong) * cos(lat2);
  float x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dLong);
  float brng = atan2(y, x) * 180/ M_PI;
  if (brng<0){
    brng+=360;
  }
  return brng;
}

/*Returns distance (in meters) between two global coordinates.
* Coordinates must be given in latitude and longitude */
float toMeters(float lat1, float lon1, float lat2, float lon2){
  float R = 6371000;
  float phi1 = lat1  *M_PI/180;
  float phi2 = lat2 *M_PI/180;
  float dphi1 = (lat2-lat1) *M_PI/180;
  float dphi2 = (lon2-lon1) *M_PI/180;

  float a = sin(dphi1/2) * sin(dphi1/2) + cos(phi1) * cos(phi2) * sin(dphi2/2) * sin(dphi2/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));

  float d = R * c;
  return d;
}

/*----------NAVIGATION ALGORITHM----------
*
*Uses sensorData.windDir, sensorData.boatDir to set sailAngle and tailAngle. 
* sailAngle and tailAngle are set in terms of servo command numbers, but are first
* calculated in terms of angle w.r.t the boat direction*/
void nShort(void) {

  //find the normal distance to the waypoint
  r[0] = wayPoints[wpNum].longitude - sensorData.longi;
  r[1] = wayPoints[wpNum].latitude - sensorData.lati;
  w[0] = cos((sensorData.windDir)*(PI/180.0));
  w[1] = sin((sensorData.windDir)*(PI/180.0));
  normr = sqrt(pow(r[0],2)+pow(r[1],2));

  //Dummy normal distance
  float oldnormr=1000;

  angleofattack = 15;

  //Optimal angle to go at if we cannot go directly to the waypoint
  //Puts us on a tack or jibe
  //Different values for top and bottom of polar plot
  optpolartop= 45;
  optpolarbot= 40;

  Serial.println("----------NAVIGATION----------");
  Serial1.println("----------NAVIGATION----------");
  Serial.print("Next Waypoint #");
  Serial.print(wpNum);
  Serial.print(": ");
  Serial.print(wayPoints[wpNum].latitude);
  Serial.print(", ");
  Serial.println(wayPoints[wpNum].longitude);

  Serial1.print("Next Waypoint #");
  Serial1.print(wpNum);
  Serial1.print(": ");
  Serial1.print(wayPoints[wpNum].latitude);
  Serial1.print(", ");
  Serial1.println(wayPoints[wpNum].longitude);

  //Reached waypoint!
  if((normr < detectionradius) && ((wpNum + 1) < numWP)){
    Serial.println("Reached waypoint #");
    Serial.print(wpNum);
    Serial.print(": ");
    Serial.print(wayPoints[wpNum].latitude);
    Serial.print(", ");
    Serial.println(wayPoints[wpNum].longitude);
    
    Serial1.println("Reached waypoint #");
    Serial1.print(wpNum);
    Serial1.print(": ");
    Serial1.print(wayPoints[wpNum].latitude);
    Serial1.print(", ");
    Serial1.println(wayPoints[wpNum].longitude);
    wpNum = wpNum+1;

    //reset variables because we have reached the old waypoint
    r[0] = wayPoints[wpNum].longitude - sensorData.longi;
    r[1] = wayPoints[wpNum].latitude - sensorData.lati;
    w[0] = cos((sensorData.windDir)*(PI/180.0));
    w[1] = sin((sensorData.windDir)*(PI/180.0));
    oldnormr=normr;
    normr = sqrt(pow(r[0],2)+pow(r[1],2));
  }

  //dir is the direction to the next waypoint from the boat
  //because we want the angle from the y axis (from the north), we take atan of adjacent over oppoosite
  float anglewaypoint=atan2(r[0],r[1])*360/(2*PI);
  //converts to 0-360
  anglewaypoint=(float)((int)anglewaypoint%360);
  anglewaypoint=anglewaypoint+360;
  anglewaypoint=(float)((int)anglewaypoint%360);

  Serial.print("Angle to the waypoint:"); Serial.println(anglewaypoint);
  Serial1.print("Angle to the waypoint:"); Serial1.println(anglewaypoint);

  float dirangle=anglewaypoint-sensorData.windDir;
  //converts to 0-360
  dirangle=(float)((int)dirangle%360);
  dirangle=dirangle+360;
  dirangle=(float)((int)dirangle%360);

  Serial.print("Angle to the waypoint w.r.t Wind:"); Serial.println(dirangle);
  Serial1.print("Angle to the waypoint w.r.t Wind:"); Serial1.println(dirangle);

  float boatDirection = sensorData.boatDir; 
 
  boatDirection=360-(boatDirection-90);
  boatDirection=(float)((int)boatDirection%360);
  boatDirection=boatDirection+360;
  boatDirection=(float)((int)boatDirection%360);
  int side;

  // Wind w.r.t the boat
  float windboat;
  windboat=sensorData.windDir-boatDirection;

  Serial.println("Setting sail and tail");
  Serial1.println("Setting sail and tail");
  
  //Up right
  if (dirangle<optpolartop && dirangle>0){
    Serial.println("Heading up right");
    Serial1.println("Heading up right");  
    sailAngle=sensorData.windDir+angleofattack;
    tailAngle=sensorData.windDir;
  }
  //Head directly to target to the right
  else if (dirangle>optpolartop && dirangle<180- optpolarbot){
    Serial.println("Heading to target to the right");
    Serial1.println("Heading to target to the right");
    sailAngle=sensorData.windDir+angleofattack;
    tailAngle=sensorData.windDir;
  }
  //Head directly to target to the right
  else if (dirangle>optpolarbot + 180 && dirangle<360 -optpolartop){
    Serial.println("Heading to target to the left");
    Serial1.println("Heading to target to the left");
    sailAngle=sensorData.windDir-angleofattack;
    tailAngle=sensorData.windDir;
  }
  //Up left
  else if (dirangle>360-optpolartop){
    Serial.println("up left");
    Serial1.println("up left");
    sailAngle=sensorData.windDir-angleofattack;
    tailAngle=sensorData.windDir;
  }   
  else if (dirangle < 180 + optpolarbot && dirangle > 180){
    Serial.println("bottom left");
    Serial1.println("bottom left");
    sailAngle=sensorData.windDir-angleofattack;
    tailAngle=sensorData.windDir;
  }
  else {
    Serial.println("bottom right");
    Serial1.println("bottom right");
    sailAngle=optpolarbot + angleofattack;
    tailAngle=optpolarbot;
  }

  Serial.print("Sail angle (0 to 360) w.r.t North: ");   Serial.println(sailAngle);
  Serial.print("Tail angle (0 to 360) w.r.t North: ");   Serial.println(tailAngle);
  Serial1.print("Sail angle (0 to 360) w.r.t North: ");   Serial1.println(sailAngle);
  Serial1.print("Tail angle (0 to 360) w.r.t North: ");   Serial1.println(tailAngle);

  //Print boat and wind direction to make sure data is consistent at this point
  Serial.print("sensorData.boatDir: ");   Serial.println(sensorData.boatDir);
  Serial.print("sensorData.windDir: ");   Serial.println(sensorData.windDir);
  Serial1.print("sensorData.boatDir: ");   Serial1.println(sensorData.boatDir);
  Serial1.print("sensorData.windDir: ");   Serial1.println(sensorData.windDir);

  //Convert sail and tail from wrt north to wrt boat
  sailAngle=sailAngle-sensorData.boatDir;
  tailAngle=tailAngle-sensorData.boatDir;
  sailAngle = int(sailAngle+360)%360;
  tailAngle = int(tailAngle+360)%360;
  if (tailAngle> 180) {tailAngle -= 360;}

  Serial.print("Sail angle (0 to 360) w.r.t Boat: ");   Serial.println(sailAngle);
  Serial.print("Tail angle (0 to 360) w.r.t Boat: ");   Serial.println(tailAngle);
  Serial1.print("Sail angle (0 to 360) w.r.t Boat: ");   Serial1.println(sailAngle);
  Serial1.print("Tail angle (0 to 360) w.r.t Boat: ");   Serial1.println(tailAngle);


  //Get servo commands from the calculated sail and tail angles
  if (sailAngle < 0) {
    sailAngle += 360;
  }

  tailAngle = tailMap(sailAngle, tailAngle);
  sailAngle = sailMap(sailAngle);

  Serial.print("Sail angle (Servo Command): ");   Serial.println(sailAngle);
  Serial.print("Tail angle (Servo Command): ");   Serial.println(tailAngle);
  Serial1.print("Sail angle (Servo Command): ");   Serial1.println(sailAngle);
  Serial1.print("Tail angle (Servo Command): ");   Serial1.println(tailAngle);

  Serial.println("");
  Serial1.println("");
  Serial.println("--------------------");
  Serial1.println("--------------------");
  Serial.println("");
  Serial1.println("");
 
} 

/*Sets servos to the sailAngle and tailAngle*/
void nServos(void) {
  tailServo.write(tailAngle);
  sailServo.write(sailAngle); 
}
