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
float detectionradius = 15; //how far away the boat marks a waypoint "reached"
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
coord_t engQuadX = {42.444612, -76.483492}; //Center of Engineering Quad
coord_t bottomStairs = {42.444240, -76.483258}; //Bottom of stairs, outside West entrance to Duffield Hall
coord_t northBottomStairs = {42.444735, -76.483275}; //Bottom of stairs, to the left of North entrance to Duffield Hall
coord_t engQuadRight = {42.4444792, -76.483244}; //Middle of the right sector, looking North, of the engineering quad

//Coordinates in and around the Cornell Sailing Center
coord_t lakeOut = {42.469386,-76.504690}; //Out in the lake, to the left of the Cornell Sailing Center
coord_t lakeOut12 = {42.469847,-76.504522}; //Out in the lake, to the left of the Cornell Sailing Center
coord_t lakeOut2 = {42.469065,-76.506674}; //Out in the lake, to the right of the Cornell Sailing Center
coord_t lakeOut3 = {42.470894,-76.504712}; //Out in the lake, to the right of the Cornell Sailing Center but further North
coord_t shore = {42.469717,-76.503341}; //Far end of the docks, to the left of the Cornell Sailing Center
coord_t shore2 = {42.470862,-76.503950}; //Beach, to the right of the Cornell Sailing Center
coord_t acrossDock = {42.465702, -76.524625}; //Across the lake, when looking from the far edge of the dock to the right of the Cornell Sailing Center
coord_t acrossBeach = {42.467918, -76.525419}; //Across the lake, when looking from the beach to the left of the Cornell Sailing Center 

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
  numWP = 2;
  wpNum = 0;

  //Set way points to desired coordinates.
  //Assignmment must be of the type coord_t.
  wayPoints[0] = acrossDock;
  wayPoints[1] = shore;

  
  //Serial prints to Serial Monitor
  Serial.println("First coordinate: ");
  Serial.print("latitude: "); Serial.println(wayPoints[0].longitude,6);
  Serial.print("longitude: "); Serial.println(wayPoints[0].latitude,6);

  
  //Serial1 print through the XBees
  Serial1.println("First coordinate: ");
  Serial1.print("latitude: "); Serial.println(wayPoints[0].longitude,6);
  Serial1.print("longitude: "); Serial.println(wayPoints[0].latitude,6);

  //Serial prints to Serial Monitor
  Serial.println("Second coordinate: ");
  Serial.print("latitude: "); Serial.println(wayPoints[1].longitude,6);
  Serial.print("longitude: "); Serial.println(wayPoints[1].latitude,6);

  
  //Serial1 print through the XBees
  Serial1.println("Second coordinate: ");
  Serial1.print("latitude: "); Serial.println(wayPoints[1].longitude,6);
  Serial1.print("longitude: "); Serial.println(wayPoints[1].latitude,6);
}


/*Returns angle (with respect to North) between two global coordinates.*/
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

/*Returns great circle distance (in meters) between two global coordinates.*/
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
  Serial.print("Detection Radius: ");Serial.println(detectionradius);

  Serial1.print("Next Waypoint #");
  Serial1.print(wpNum);
  Serial1.print(": ");
  Serial1.print(wayPoints[wpNum].latitude);
  Serial1.print(", ");
  Serial1.println(wayPoints[wpNum].longitude);
  Serial1.print("Distance to waypoint: ");Serial1.println(normr);
  Serial1.print("Detection Radius: ");Serial1.println(detectionradius);

  //Reached waypoint!
  if((normr < detectionradius) && ((wpNum + 1) < numWP)){
    Serial.println("");
    Serial.println("");
    Serial.println("");
    Serial.println("----------");
    Serial.println("REACHED WAYPOINT");
    Serial.println("----------");
    Serial.println("");
    Serial.println("");
    Serial.println("");
    
    
    Serial.println("Reached waypoint #");
    Serial.print(wpNum);
    Serial.print(": ");
    Serial.print(wayPoints[wpNum].latitude);
    Serial.print(", ");
    Serial.println(wayPoints[wpNum].longitude);

    Serial1.println("");
    Serial1.println("");
    Serial1.println("");
    Serial1.println("----------");
    Serial1.println("REACHED WAYPOINT");
    Serial1.println("----------");
    Serial1.println("");
    Serial1.println("");
    Serial1.println("");
    
    Serial1.println("Reached waypoint #");
    Serial1.print(wpNum);
    Serial1.print(": ");
    Serial1.print(wayPoints[wpNum].latitude);
    Serial1.print(", ");
    Serial1.println(wayPoints[wpNum].longitude);
    wpNum += 1 ;

    //reset variables because we have reached the old waypoint
    r[0] = wayPoints[wpNum].longitude - sensorData.longi;
    r[1] = wayPoints[wpNum].latitude - sensorData.lati;
    w[0] = cos((sensorData.windDir)*(PI/180.0));
    w[1] = sin((sensorData.windDir)*(PI/180.0));
    coord_t currentPosition = {sensorData.lati, sensorData.longi};
    normr = havDist(wayPoints[wpNum], currentPosition);

    digitalWrite(redLED1, HIGH);
    digitalWrite(redLED2, HIGH);
    digitalWrite(yellowLED, HIGH);
    digitalWrite(blueLED, HIGH);
    digitalWrite(greenLED, HIGH);

    for (int i=0; i<numBoatDirReads; i++){
      boatDirections[i] = 0;
    }

    delay(8000); //Wait 8 seconds after hitting a waypoint
    
  }

    digitalWrite(redLED1, LOW);
    digitalWrite(redLED2, LOW);
    digitalWrite(yellowLED, LOW);
    digitalWrite(blueLED, LOW);
    digitalWrite(greenLED, HIGH);

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

//  boat initially facing right
  if (dirangle<180) {
    //Up right
    if (dirangle<optpolartop && dirangle>0){
      Serial.println("Right up right");
      Serial1.println("Right up right");  
      sailAngle= sensorData.windDir - (sensorData.boatDir - optpolartop) - angleofattack; //sensorData.windDir+angleofattack ;
      tailAngle= sensorData.windDir - (sensorData.boatDir - optpolartop);
    }
    //Head directly to target to the right
    else if (dirangle>optpolartop && dirangle<180- optpolarbot){
      Serial.println("Right to target to the right");
      Serial1.println("Right to target to the right");
      sailAngle=sensorData.windDir+angleofattack;
      tailAngle=sensorData.windDir;
    }
    //Head directly to target to the left
    else if (dirangle>optpolarbot + 180 && dirangle<360 -optpolartop){
      Serial.println("Right to target to the left");
      Serial1.println("Right to target to the left");
      sailAngle=sensorData.windDir-angleofattack;
      tailAngle=sensorData.windDir;
    }
    //Up left
    else if (dirangle>360-optpolartop){
      Serial.println("Right up left");
      Serial1.println("Right up left");
      sailAngle= sensorData.windDir + (sensorData.boatDir - optpolartop) + angleofattack; //sensorData.windDir+angleofattack;
      tailAngle= sensorData.windDir + (sensorData.boatDir - optpolartop);
    }   
    //bottom left    
    else if (dirangle < 180 + optpolarbot && dirangle > 180){
      Serial.println("Right bottom left");
      Serial1.println("Right bottom left");
      sailAngle= sensorData.windDir - (sensorData.boatDir - optpolarbot) + angleofattack; //sensorData.windDir+angleofattack;
      tailAngle= sensorData.windDir - (sensorData.boatDir - optpolarbot);
    }
    //bottom right
    else {
      Serial.println("Right bottom right");
      Serial1.println("Right bottom right");
      sailAngle= sensorData.windDir + (sensorData.boatDir - optpolarbot) - angleofattack;
      tailAngle= sensorData.windDir + (sensorData.boatDir - optpolarbot);
    }
  }
  //boat facing to left  
  else{
    //Up right
    if (dirangle<optpolartop && dirangle>0){
      Serial.println("Left up right");
      Serial1.println("Left up right");  
      sailAngle=sensorData.windDir-angleofattack ;
      tailAngle=sensorData.windDir;
    }
    //Head directly to target to the right
    else if (dirangle>optpolartop && dirangle<180- optpolarbot){
      Serial.println("Left to target to the right");
      Serial1.println("Left to target to the right");
      sailAngle=sensorData.windDir+angleofattack;
      tailAngle=sensorData.windDir;
    }
    //Head directly to target to the left
    else if (dirangle>optpolarbot + 180 && dirangle<360 -optpolartop){
      Serial.println("Left to target to the left");
      Serial1.println("Left to target to the left");
      sailAngle=sensorData.windDir-angleofattack;
      tailAngle=sensorData.windDir;
    }
    //Up left
    else if (dirangle>360-optpolartop){
      Serial.println("Left up left");
      Serial1.println("Left up left");
      sailAngle=sensorData.windDir-angleofattack;
      tailAngle=sensorData.windDir;
    }   
    //bottom left    
    else if (dirangle < 180 + optpolarbot && dirangle > 180){
      Serial.println("Left bottom left");
      Serial1.println("Left bottom left");
      sailAngle=sensorData.windDir-angleofattack;
      tailAngle=sensorData.windDir;
    }
    //bottom right
    else {
      Serial.println("Left bottom right");
      Serial1.println("Left bottom right");
      sailAngle=sensorData.windDir - angleofattack;
      tailAngle=sensorData.windDir;
    }
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

/*
/** Returns a double:
*   2 indicates no object
*   0 to 1 indicates object on the starboard (closer to 1 = closer to edge of pixy cam view)
*  -1 to 0 indicates object on the port (closer to -1 = closer to edge of pixy cam view)
*   NOTE: This only detects objects set to
*   signature 1 on the pixy cam */
double getObjects() {
    uint16_t blocks = pixy.getBlocks();

    if (blocks) {
        for (int j = 0; j < blocks, j++) {
            if (pixy.blocks[j].signature == 1) {
                int32_t xLocation = pixy.blocks[j].x; // range: 0 to 319
                double half = xLocation / 2.0;
                return (half / 159.5) - 1.0;
            }
        }
    }
    
    //otherwise there are no objects of interest
    return 2.0;
}
*/

