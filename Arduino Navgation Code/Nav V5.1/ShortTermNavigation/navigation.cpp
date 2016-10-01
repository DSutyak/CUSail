#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include "sensors.h"
#include "navigation.h"

//Define Servo types for Sail and Tail
Servo tailServo;
Servo sailServo;

/*----------Navigation Variables----------*/
int wpNum; //the current waypoint's number in the wayPoints array
int numWP; //total number of waypoints on current course
float detectionradius=5; //how far away the boat marks a waypoint "reached"
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
double havDist(coord_t  first, coord_t second) {
  double x = first.longitude;
  double y = first.latitude;

  double x1 = second.longitude;
  double y1 = second.latitude;

  const double conversion = M_PI / 180;// term to convert from degrees to radians
  const double r = 6371.0;//radius of earth in km
  x = x * conversion;// convert x to radians
  y = y * conversion;// convert y to radians
  x1 = x1 * conversion;
  y1 = y1 * conversion;
    
  double half1 = (y-y1) / 2;
  double half2 = (x-x1) / 2;
        
  double part1 = sin(half1) * sin(half1) + cos(y) * cos(y1) * sin(half2) * sin(half2);
  double part2 = sqrt(part1);
  double distance = 2 * r * asin(part2);// distance is in km due to units of earth's radius

  distance = (distance*1000);

  if (distance<30) {
  double longDiff = wayPoints[wpNum].longitude - sensorData.longi;
  double latDiff  = wayPoints[wpNum].latitude - sensorData.lati;
  distance = sqrt(pow(longDiff,2)+pow(latDiff,2));
  }

  return distance;
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
  coord_t currentPosition = {sensorData.lati, sensorData.longi};
  normr = havDist(wayPoints[wpNum], currentPosition);

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
  Serial.print("Distance to waypoint: ");Serial.println(normr);

  Serial1.print("Next Waypoint #");
  Serial1.print(wpNum);
  Serial1.print(": ");
  Serial1.print(wayPoints[wpNum].latitude);
  Serial1.print(", ");
  Serial1.println(wayPoints[wpNum].longitude);
  Serial.print("Distance to waypoint: ");Serial1.println(normr);

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
    coord_t currentPosition = {sensorData.lati, sensorData.longi};
    normr = havDist(wayPoints[wpNum], currentPosition); 
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
