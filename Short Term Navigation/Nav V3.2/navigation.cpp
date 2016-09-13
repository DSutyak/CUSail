include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include "sensors.h"
#include "navigation.h"


\\Define Servos
Servo tailServo;
Servo sailServo;


float prevErr;
coord_t wayPoint[MAX_WAYPOINTS];
int wpNum;
int totalWP;
typedef int mwSize;


// The polar plot gives the expected speed of boat, given its heading with respect to the wind
// 0 degree heading represents sailing directly into the wind
float polarPlot [361] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0.1712, 0.17325, 0.17742, 0.18336, 0.19064, 0.19882, 0.20757, 0.21661,
                         0.22575, 0.23487, 0.24389, 0.25275, 0.26143, 0.26991, 0.27818, 0.28623, 0.29407, 0.3017, 0.30912, 0.31634,
                         0.32337, 0.33021, 0.33687, 0.34335, 0.34966, 0.35581, 0.36179, 0.36761, 0.37329, 0.37882, 0.3842, 0.38944,
                         0.39455, 0.39952, 0.40437, 0.40909, 0.41368, 0.41815, 0.42251, 0.42675, 0.43087, 0.43488, 0.43878, 0.44257,
                         0.44626, 0.44984, 0.45332, 0.4567, 0.45998, 0.46315, 0.46624, 0.46922, 0.47211, 0.47491, 0.47761, 0.48023,
                         0.48275, 0.48518, 0.48753, 0.48979, 0.49196, 0.49405, 0.49605, 0.49797, 0.4998, 0.50155, 0.50322, 0.50481,
                         0.50632, 0.50774, 0.50909, 0.51036, 0.51155, 0.51267, 0.5137, 0.51466, 0.51555, 0.51636, 0.51709, 0.51775,
                         0.51833, 0.51884, 0.51927, 0.51964, 0.51992, 0.52014, 0.52028, 0.52035, 0.52035, 0.52028, 0.52013, 0.51991,
                         0.51962, 0.51926, 0.51883, 0.51833, 0.51775, 0.51711, 0.51639, 0.5156, 0.51474, 0.51381, 0.51281, 0.51173,
                         0.51059, 0.50937, 0.50808, 0.50672, 0.50529, 0.50378, 0.5022, 0.50055, 0.49882, 0.49702, 0.49514, 0.49319,
                         0.49117, 0.48907, 0.48689, 0.48463, 0.4823, 0.47989, 0.4774, 0.47483, 0.47218, 0.46945, 0.46663, 0.46373,
                         0.46075, 0.46373, 0.46663, 0.46945, 0.47218, 0.47483, 0.4774, 0.47989, 0.4823, 0.48463, 0.48689, 0.48907,
                         0.49117, 0.49319, 0.49514, 0.49702, 0.49882, 0.50055, 0.5022, 0.50378, 0.50529, 0.50672, 0.50808, 0.50937,
                         0.51059, 0.51173, 0.51281, 0.51381, 0.51474, 0.5156, 0.51639, 0.51711, 0.51775, 0.51833, 0.51883, 0.51926,
                         0.51962, 0.51991, 0.52013, 0.52028, 0.52035, 0.52035, 0.52028, 0.52014, 0.51992, 0.51964, 0.51927, 0.51884,
                         0.51833, 0.51775, 0.51709, 0.51636, 0.51555, 0.51466, 0.5137, 0.51267, 0.51155, 0.51036, 0.50909, 0.50774,
                         0.50632, 0.50481, 0.50322, 0.50155, 0.4998, 0.49797, 0.49605, 0.49405, 0.49196, 0.48979, 0.48753, 0.48518,
                         0.48275, 0.48023, 0.47761, 0.47491, 0.47211, 0.46922, 0.46624, 0.46315, 0.45998, 0.4567, 0.45332, 0.44984,
                         0.44626, 0.44257, 0.43878, 0.43488, 0.43087, 0.42675, 0.42251, 0.41815, 0.41368, 0.40909, 0.40437, 0.39952,
                         0.39455, 0.38944, 0.3842, 0.37882, 0.37329, 0.36761, 0.36179, 0.35581, 0.34966, 0.34335, 0.33687, 0.33021,
                         0.32337, 0.31634, 0.30912, 0.3017, 0.29407, 0.28623, 0.27818, 0.26991, 0.26143, 0.25275, 0.24389, 0.23487,
                         0.22575, 0.21661, 0.20757, 0.19882, 0.19064, 0.18336, 0.17742, 0.17325, 0.1712, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                        };

// Navigation Variables
float nextHeading;
float error;
float command;
int alpha;
float n;
float normr;
float v_T_R_max;
float v_T_L_max;
float v_T_R;
float v_T_L;
float thetaBoat_R_max;
float thetaBoat_L_max;
float vB[2];
float r[2];
float w[2];
float d1;
float d2;
int opt = 0;
int delalpha = 1;
float sailAngle;
float tailAngle;
float waypointsize;
float jibing;
float tackpointx;
float tackpointy;
float avgWind;
float prevNormr;
float timeAway;
int waypointnumber;
double boatLoc[2];
double wayPoints[MAX_WAYPOINTS];
float windDir, boatDir;

/* Initialize servos */
void initServos(void) {
  tailServo.attach(tailServoPin);
  sailServo.attach(sailServoPin);
  tailAngle = 0;
  sailAngle = 0;
}

/*Initializes navigation algorithm */
void initNavigation(void) {
  prevErr = 0;
  wpNum = 0;
  totalWP = 0;
  jibing = 0;
  tackpointx = 0;
  tackpointy = 0;
  avgWind = 0;
}

/* Sets/defines waypoints */
void setWaypoints(void) {

  //coordinates for navigation testing on the Engineering Quad, Cornell University
  coord_t olin = {42.445457, -76.484322}; //South entrance of Olin Hall, Cornell University
  coord_t hollister = {42.444259, -76.484435}; //East entrance of Hollister Hall, Cornell University
  coord_t sundial = {42.4448087,-76.4834094}; //Sundial on the Engineering Quad, Cornell University
  coord_t outsideThurston = {42.444228, -76.483666};  //North entrance of Thurston Hall, Cornell University
  coord_t engQuadX = {42.444591, -76.483498};  //Center of the Engineering Quad, Cornell University
  coord_t westDuffield = {42.444240, -76.483258};  //West entrance of Duffield Hall, Cornell University
  
  
  //coordinates for navigation testing on Cayuga Lake, off from the Cornell Sailing Center
  coord_t cayugaLake1 = {42.468829, -76.503439}; \\The dock
  coord_t cayugaLake2 = {42.469057, -76.502932};
  

  //set waypoints here
  //waypoint[n] = <name of coord_t variable that holds coordinates>
  wayPoint[0] = cayugaLake1;
  wayPoint[1] =  cayugaLake1;

  //Print inforamation of coordinates to PC (Serial) and through Xbee Modules (Serial 1)

  Serial.println("First coordinate: ");
  Serial.print("latitude: "); Serial.println(wayPoint[1].longitude, 6);
  Serial.print("longitude: "); Serial.println(wayPoint[2].latitude, 6);
  Serial.println("\nSecond coordinate: ");
  Serial.print("latitude: "); Serial.println(wayPoint[2].longitude, 6);
  Serial.print("longitude: "); Serial.println(wayPoint[2].latitude, 6);

  Serial1.println("First coordinate: ");
  Serial1.print("latitude: "); Serial1.println(wayPoint[1].longitude, 6);
  Serial1.print("longitude: "); Serial1.println(wayPoint[2].latitude, 6);
  Serial1.println("\nSecond coordinate: ");
  Serial1.print("latitude: "); Serial1.println(wayPoint[2].longitude, 6);
  Serial1.print("longitude: "); Serial1.println(wayPoint[2].latitude, 6);

  totalWP = 2;
  wpNum = 0; //Waypoint to head to first
  waypointsize = 2 * totalWP;
}


/*Given two gps coordinates, finds the angle between them with respect to true North */

float angleToTarget(float lat1, float long1, float lat2, float long2) {
  lat1 = lat1 * M_PI / 180;
  lat2 = lat2 * M_PI / 180;
  float dLong = (long2 - long1) * M_PI / 180;
  float y = sin(dLong) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLong);
  float brng = atan2(y, x) * 180 / M_PI;
  if (brng < 0) {
    brng += 360;
  }
  
  return brng;
}

/*Given two gps coordinates, finds the meter distance between them*/
float toMeters(float lat1, float lon1, float lat2, float lon2) {
  float R = 6371000;
  float phi1 = lat1  * M_PI / 180;
  float phi2 = lat2 * M_PI / 180;
  float dphi1 = (lat2 - lat1) * M_PI / 180;
  float dphi2 = (lon2 - lon1) * M_PI / 180;

  float a = sin(dphi1 / 2) * sin(dphi1 / 2) + cos(phi1) * cos(phi2) * sin(dphi2 / 2) * sin(dphi2 / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));

  float d = R * c;
  return d;
}

/*The Short term Navigation Algorithm*/
void nShort(void) {
  //   Inputs:
  //
  //     wayPoints - a double array representing the boat's destination points, as
  //                  set by the long course code. Within array, values at even indices are
  //                  latitude, and odd indices are longitude
  //      sensorData - a size 2 double array containing boat's current latitude and longitude
  //                      sensorData[0] is longitude
  //                      sensorData[1] is latitude
  //      windDir - a float between 0 and 360 that represents the current wind direction
  //      boatDir - a float between 0 and 360 that represents the current boat direction
  //      wpNum - index of the longitude value within wayPoints of the way point that
  //              the boat is currently traveling towards
  //      prevErr - difference between boat's last desired heading and its actually heading
  //   
  //  Outputs:
  //
  //      angles - a size 4 float array containing rudder angle (float between 0 and 360) at index 0,
  //               sail angle (float between 0 and 360) at index 1, the updated value of prevErr after this
  //               calculation at index 3, and the updated value of wpNum at index 4
  //
  // Note 0: This function has two modes; one that works with the simulation made by Sara Gregg (Mechanical Engineering , 2016),
  //         and one that runs with the boats sensor array
  
  // Note 1: Should be initializing wpNum and prevErr as 0 within the simulation (mode 1), and then
  //         updating them after each call to the short course algorithm (mode 0)
  //
  // Note 2: parts of the code that usually interact with Arduino sensors have been commented out, and replaced
  //         with code to interact with input parameters representing sensor information


  int simulation = 1; //The simulation version of the code is mode 1 (Please see Note 0 above)
  
  int mode = 0; //Set the appropriate mode
  
  float wpErr; //Waypoint error

  //Simulation Mode:
  
  if (mode == simulation) {
    // TO DO: see which of these variables are still in use
    float polarPlot [361] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0.1712, 0.17325, 0.17742, 0.18336, 0.19064, 0.19882, 0.20757, 0.21661,
                             0.22575, 0.23487, 0.24389, 0.25275, 0.26143, 0.26991, 0.27818, 0.28623, 0.29407, 0.3017, 0.30912, 0.31634,
                             0.32337, 0.33021, 0.33687, 0.34335, 0.34966, 0.35581, 0.36179, 0.36761, 0.37329, 0.37882, 0.3842, 0.38944,
                             0.39455, 0.39952, 0.40437, 0.40909, 0.41368, 0.41815, 0.42251, 0.42675, 0.43087, 0.43488, 0.43878, 0.44257,
                             0.44626, 0.44984, 0.45332, 0.4567, 0.45998, 0.46315, 0.46624, 0.46922, 0.47211, 0.47491, 0.47761, 0.48023,
                             0.48275, 0.48518, 0.48753, 0.48979, 0.49196, 0.49405, 0.49605, 0.49797, 0.4998, 0.50155, 0.50322, 0.50481,
                             0.50632, 0.50774, 0.50909, 0.51036, 0.51155, 0.51267, 0.5137, 0.51466, 0.51555, 0.51636, 0.51709, 0.51775,
                             0.51833, 0.51884, 0.51927, 0.51964, 0.51992, 0.52014, 0.52028, 0.52035, 0.52035, 0.52028, 0.52013, 0.51991,
                             0.51962, 0.51926, 0.51883, 0.51833, 0.51775, 0.51711, 0.51639, 0.5156, 0.51474, 0.51381, 0.51281, 0.51173,
                             0.51059, 0.50937, 0.50808, 0.50672, 0.50529, 0.50378, 0.5022, 0.50055, 0.49882, 0.49702, 0.49514, 0.49319,
                             0.49117, 0.48907, 0.48689, 0.48463, 0.4823, 0.47989, 0.4774, 0.47483, 0.47218, 0.46945, 0.46663, 0.46373,
                             0.46075, 0.46373, 0.46663, 0.46945, 0.47218, 0.47483, 0.4774, 0.47989, 0.4823, 0.48463, 0.48689, 0.48907,
                             0.49117, 0.49319, 0.49514, 0.49702, 0.49882, 0.50055, 0.5022, 0.50378, 0.50529, 0.50672, 0.50808, 0.50937,
                             0.51059, 0.51173, 0.51281, 0.51381, 0.51474, 0.5156, 0.51639, 0.51711, 0.51775, 0.51833, 0.51883, 0.51926,
                             0.51962, 0.51991, 0.52013, 0.52028, 0.52035, 0.52035, 0.52028, 0.52014, 0.51992, 0.51964, 0.51927, 0.51884,
                             0.51833, 0.51775, 0.51709, 0.51636, 0.51555, 0.51466, 0.5137, 0.51267, 0.51155, 0.51036, 0.50909, 0.50774,
                             0.50632, 0.50481, 0.50322, 0.50155, 0.4998, 0.49797, 0.49605, 0.49405, 0.49196, 0.48979, 0.48753, 0.48518,
                             0.48275, 0.48023, 0.47761, 0.47491, 0.47211, 0.46922, 0.46624, 0.46315, 0.45998, 0.4567, 0.45332, 0.44984,
                             0.44626, 0.44257, 0.43878, 0.43488, 0.43087, 0.42675, 0.42251, 0.41815, 0.41368, 0.40909, 0.40437, 0.39952,
                             0.39455, 0.38944, 0.3842, 0.37882, 0.37329, 0.36761, 0.36179, 0.35581, 0.34966, 0.34335, 0.33687, 0.33021,
                             0.32337, 0.31634, 0.30912, 0.3017, 0.29407, 0.28623, 0.27818, 0.26991, 0.26143, 0.25275, 0.24389, 0.23487,
                             0.22575, 0.21661, 0.20757, 0.19882, 0.19064, 0.18336, 0.17742, 0.17325, 0.1712, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                            };
    float nextHeading;
    float error;
    float command;
    mwSize alpha;
    float n;
    float normr;
    float v_T_R_max;
    float v_T_L_max;
    float v_T_R;
    float v_T_L;
    float thetaBoat_R_max;
    float thetaBoat_L_max;
    float vB[2];
    float r[2];
    float d1;
    float d2;
    mwSize opt = 0;
    mwSize delalpha = 1;
    float rudderAngle;
    float sailAngle;
    float tailangle;
    wpErr = 3;
  }
  else {

  //Sensor Array Mode:

    wpErr = 4;

    //Print inforamation of initialization to PC (Serial) and through Xbee Modules (Serial 1)
    Serial.println("\n --- Short Term Navigation ---");
    Serial.print("Current location: "); Serial.print(sensorData.lati, 6);
    Serial.print(", ");  Serial.println(sensorData.longi, 6);
    Serial.print("Next waypoint #"); Serial.print(wpNum);
    Serial.print(": ");
    Serial.print(wayPoint[wpNum].latitude, 6);
    Serial.print(", ");
    Serial.println(wayPoint[wpNum].longitude, 6);

    Serial1.println("\n --- Short Term Navigation ---");
    Serial1.print("Current location: "); Serial1.print(sensorData.lati, 6);
    Serial1.print(", ");
    Serial1.println(sensorData.longi, 6);
    Serial1.print("Next waypoint #"); Serial1.print(wpNum);
    Serial1.print(": ");
    Serial1.print(wayPoint[wpNum].latitude, 6);
    Serial1.print(", ");
    Serial1.println(wayPoint[wpNum].longitude, 6);
  }

  //Setting up variables

  //Simulation Mode:
  
  if (mode == simulation) {
    //coming from north=0, E=90, S=180, W=270
    windDir = 360 - (windDir - 90);
    printf("windir: %f\n", windDir);
    int waypointnumber = floor(wpNum);
    printf("weird wind direction: %f\n", waypointnumber);
    printf("checkingwaypoint normr: %f, wpNum: %f\n", normr, waypointnumber);
  }
  else {
    
    //Sensor Array Mode:
    windDir = sensorData.windDir;
  
  }

  // Calculates distance vector from boat to target, normr, using latitude and longitude from sensor
  if (mode == simulation) {
    r[0] = wayPoints[waypointnumber] - boatLoc[0];
    r[1] = wayPoints[waypointnumber + 1] - boatLoc[1];
    //w[0] = cos((windDir)*(PI/180.0));
    //w[1] = sin((windDir)*(PI/180.0));
    normr = sqrt(pow(r[0], 2) + pow(r[1], 2));
  }
  else {
    r[1] = wayPoint[wpNum].latitude - sensorData.lati;
    r[0] = wayPoint[wpNum].longitude - sensorData.longi;
    //w[0] = cos((sensorData.windDir)*(PI/180.0));
    //w[1] = sin((sensorData.windDir)*(PI/180.0));
    normr = toMeters(sensorData.lati, sensorData.longi, wayPoint[wpNum].latitude, wayPoint[wpNum].longitude);
    Serial.print("   distance to waypoint: "); Serial.println(normr);
    Serial1.print("   distance to waypoint: "); Serial1.println(normr);
  }

  // oldnormr is a way to check if a waypoint was reached in this iteration
  //    initially set to 30, arbitrary (large) distance
  //    if a waypoint is reached, set oldnormr to normr, distance to waypoint that was just reached
  //        oldnormr will be < 3
  //    later, can check: if oldnormr < 3, a waypoint was just reached; otherwise it is still 30 
  //    this is needed to later reset variables for jibing code or something ??
  // TO DO: replace this with bool newWP initially set to 0 (false) and set to 1 (true) if wp is reached
  float oldnormr = 30;

  // check if waypoint reached
  // TO DO: figure out best error (set to 3 for simulation)
  if (normr < wpErr) {
    if (mode == simulation) {
      printf("reached waypoint\n");
      printf("new wpnum: %f\n", waypointnumber);

      // if there is another waypoint, reset variables
      if ((waypointnumber + 2) < waypointsize) { 
        waypointnumber += 2;
        // TO DO: figure out if any other variables should be reset here
        r[0] = wayPoints[waypointnumber] - boatLoc[0];
        r[1] = wayPoints[waypointnumber + 1] - boatLoc[1];
        //w[0] = cos((windDir)*(PI/180.0));
        //w[1] = sin((windDir)*(PI/180.0));
        oldnormr = normr;
        normr = sqrt(pow(r[0], 2) + pow(r[1], 2));
        printf("new normr: %f\n", normr );
      }
      else {
        printf("no more waypoints\n");
      }
    }
    else {
      Serial.print("\n   ----> WAYPOINT #"); Serial.print(wpNum); Serial.println(" REACHED\n");
      Serial1.print("\n   ----> WAYPOINT #"); Serial1.print(wpNum); Serial1.println(" REACHED\n");
      digitalWrite(blueLED, HIGH);

      // if there is another waypoint, reset variables
      if ((wpNum + 1) < totalWP) {
        wpNum++;
        //reset variables because we are close to a waypoint
        r[1] = wayPoint[wpNum].latitude - sensorData.lati;   //sensorData[0]
        r[0] = wayPoint[wpNum].longitude - sensorData.longi; //sensorData[1]
        w[0] = cos((sensorData.windDir) * (PI / 180.0));
        w[1] = sin((sensorData.windDir) * (PI / 180.0));
        oldnormr = normr;
        normr = toMeters(sensorData.lati, sensorData.longi, wayPoint[wpNum].latitude, wayPoint[wpNum].longitude);
        Serial.print("New waypoint #"); Serial.print(wpNum);
        Serial.print(": "); Serial.print(wayPoint[wpNum].latitude, 6);
        Serial.print(", "); Serial.println(wayPoint[wpNum].longitude, 6);
        Serial.print("Distance to new waypoint: "); Serial.println(normr, 6);
        Serial1.print("New waypoint #"); Serial1.print(wpNum);
        Serial1.print(": "); Serial1.print(wayPoint[wpNum].latitude, 6);
        Serial1.print(", "); Serial1.println(wayPoint[wpNum].longitude, 6);
        Serial1.print("Distance to new waypoint: "); Serial1.println(normr, 6);
      }
      else {
        Serial.println("there are no more waypoints");
        Serial1.println("there are no more waypoints");
      }
    }
  }

  // END --- where I understand what is going on / have cleaned up a little

  /*******************************
   *  Next heading code start  
   */

  //optimal angle to go at if we cannot go directly to the waypoint
  //puts us on a tack or jibe
  //different values for top and bottom of polar plot
  float optpolartop=45;
  float optpolarbot=40;
  float windPushError=5;
  //bottom vs top
  //dir is the direction to the next waypoint from the boat

  float anglewaypoint;
  //because we want the angle from the y axis (from the north) we take atan of adjacent over oppoosite
  if (mode == simulation) {
    printf("boat at: %f,%f\n",boatLoc[0],boatLoc[1]);
    anglewaypoint=atan2(r[0],r[1])*360/(2*PI);
  }
  else {
    anglewaypoint = angleToTarget(sensorData.lati, sensorData.longi, wayPoint[wpNum].latitude, wayPoint[wpNum].longitude);  
  }
  
  //convert anglewaypoint to 0-360F
  anglewaypoint = (float)((int)anglewaypoint % 360);
  anglewaypoint = anglewaypoint + 360;
  anglewaypoint = (float)((int)anglewaypoint % 360);

  //angle from the wind to the waypoint vector. this is used to determine if we can sail to the next heading
  //or if we have to tack/jibe

  float dirAngle = anglewaypoint - sensorData.windDir;
  //convert dirAngle to 0-360
  dirAngle = (float)((int)dirAngle % 360);
  dirAngle = dirAngle + 360;
  dirAngle = (float)((int)dirAngle % 360);
  
  if (mode == simulation) {
    // printf("angle north to waypoint: %f\n",anglewaypoint);
    // printf("anglewaypoint: %f anglewaypoint",anglewaypoint);
    // printf("dirAngle: %f\n",dirAngle);
    printf("jibing: %d \n", jibing);

    boatDir=360-(boatDir-90);
    boatDir=(float)((int)boatDir%360);
    boatDir=boatDir+360;
    boatDir=(float)((int)boatDir%360);
    printf("boat orientation: %f\n",boatDir);
  }
  else { 
    Serial.print("angle to waypoint: ");  Serial.println(anglewaypoint); 
    Serial1.print("angle to waypoint: "); Serial1.println(anglewaypoint);
    Serial.print("direction angle: "); Serial.println(dirAngle);
    Serial1.print("direction angle: "); Serial1.println(dirAngle);
    
    // TO DO: jibing print function
    Serial.print("jibing: "); Serial.println(jibing);
    Serial1.print("jibing: "); Serial1.println(jibing);

    boatDir = sensorData.boatDir;
  }

  // right=0, left =1
  int side;

  // TO DO (maybe?): add these print statements ??
  
  //we have to port tack into the wind
  //right up
  if (dirAngle < optpolartop) {
    // printf("right up port tack");
    nextHeading = optpolartop;
  }
  //we can sail directly to the target
  //right side
  else if (dirAngle < (180 - optpolarbot)) {
    // printf("right side");
    nextHeading = dirAngle;
  }
  //we have to port jibe
  //right down
  else if (dirAngle < (180)) {
    // printf("right down port jibe");
    nextHeading = 180 - optpolarbot;
  }
  //we have to starboard jibe
  //left down
  else if (dirAngle < (180 + optpolarbot)) {
    // printf("left down starboard jibe");
    nextHeading = 180 + optpolarbot;
  }
  //can sail directly to the target
  //left side
  else if (dirAngle < (360 - optpolartop)) {
    // printf("left side");
    nextHeading = dirAngle;
  }
  //starboard tack
  //left up
  else {
    // printf("left up starboard tack");
    nextHeading = 360 - optpolartop;
  }

  // printf("next heading wrt wind:%f\n",nextHeading);
  //convert back to wrt North
  nextHeading += windDir;
  
  //convert nextHeading to 0-360 (wrap around after adding windDir)
  nextHeading = (float)((int)nextHeading % 360);
//  nextHeading += 360;
//  nextHeading = (float)((int)nextHeading % 360);

  //convert nextHeading to -180 to 180
  if (nextHeading > 180) {
    nextHeading = nextHeading - 360;
  }

  // convert windDir, boatDir to -180 to 180
  if (windDir > 180) {
    windDir -= 360;
  }
  if (boatDir > 180) {
    boatDir -= 360;
  }


  // TO DO: delete the following lines if sure they are not needed
  //convert to counter clockwise direction
  // nextHeading=-1*nextHeading;
  // sensorData.windDir=-1*sensorData.windDir;
  // boatDir=-1*boatDir;


  // printf("next heading:%f\n",nextHeading);

  // TO DO: figure out how to use these?
//  int headingT[23] = {57, 62, 67, 72, 77, 82, 87, 92, 97, 102, 107, 112, 117, 122, 126, 131, 135, 139, 143, 148, 153, 158, 163};
//  int sailT[23] =   {35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 131, 137, 144, 150};
//  int tailT[23] =   {10, 11, 11, 12, 12, 12, 13, 13, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15};

  

  //
  
  float turnradius = 5;
  //greater means turn left
  float differenceheading = boatDir - nextHeading;
  float boatwind = boatDir - windDir;
  float tailAngleOfAttack = 15;

  float headingwind = nextHeading - windDir;
  if (headingwind > 180) {
    headingwind -= 360;
  }
  if (headingwind < -180) {
    headingwind += 360;
  }

  if (boatwind > 180) {
    boatwind -= 360;
  }
  if (boatwind < -180) {
    boatwind += 360;
  }

  float angleofattack = 15;


  //convert to counter clockwise direction


  //sail angle directly into the wind
  float windboat = windDir - boatDir;



  //checking what manuever



  //if we are trying to go directly upwind or downwind

  //JIBE CODE
  //TODOneed to make sure boat doesnt go off course


  //jibing 0: head directly there
  //jibing 1: turn back to get on heading
  //jibing 2: head left of the wind upwind
  //jibing 3: head right of the wind upwind
  //jibing 4: head left of the wind downwind
  //jibing 5: head right of the wind downwind
  //jibing 6: jibe left
  //jibing 7: jibe right

  //printf("normr: %f \n", normr);
  //printf("prevNormr: %f \n", prevNormr);
  if (normr>prevNormr){ // if the boat is farther away from wp than it was last time, increase timeAway by 1
    timeAway+=1;
  }
  else{
    timeAway=0;
  }
  if (timeAway>20){
    jibing=0;
  }
  prevNormr=normr;

  //printf("waypoints at: %f, %f\n", wayPoints[wpNum],wayPoints[wpNum]);

  //COMMENT OR UNCOMMENT DEPENDING ON IF TESTING USING GPS COORDINATES OR NOT
  //float distance = toMeters(tackpointx, tackpointy, sensorData.lati, sensorData.longi);
  float distance = toMeters(tackpointy, tackpointx, sensorData.lati, sensorData.longi);
  //float distance=pow(sensorData[0]-tackpointx,2)+pow(sensorData[1]-tackpointy,2);
  //float distance=pow(sensorData.lati-tackpointy,2)+pow(sensorData.longi-tackpointx,2);

  Serial.print("distance until tack: "); Serial.println(distance);
  Serial1.print("distance until tack: "); Serial1.println(distance);
  if (dirAngle > 180) {
    dirAngle -= 360;
  }
  float jibedistance = 10; //70;
  float jibeangle = 10;
  // printf("tacking point: %f, %f\n",tackpointx,tackpointy);
  // printf("dirAngle: %f\n", dirAngle);

  


  if (jibing == 0 || jibing == 1 || oldnormr < wpErr) {
    //close to last waypoint
    if ((wpNum + 1) >= totalWP && ( normr < wpErr || oldnormr < wpErr)) {
      //     if((waypointnumber + 2) >= wpsize && ( normr<3 || oldnormr<3)){
      tackpointy = sensorData.lati;
      tackpointx = sensorData.longi;
      Serial.print("close to last waypoint, setting jibing\n");
      Serial1.print("close to last waypoint, setting jibing\n");
      // printf("close to last waypoint, setting jibing\n");
      if (boatwind > 0) {
        jibing = 3;
      }
      else {
        jibing = 2;
      }
    }
      
    //reset jibing to new heading
    else {
      if (fabsf(dirAngle) < optpolartop) {

        //head to the left of the wind
        if (boatwind < 0) {
          Serial.print("set tack to 2\n"); // printf("set tack to 2\n");
          jibing = 2;
        }
        //head to the right of the wind
        else {
          Serial.print("set tack to 3\n");
          jibing = 3;
        }
      }
      else if (fabsf(dirAngle) > 180 - optpolarbot) {
        if (boatwind < 0) {
          Serial.print("set tack to 4\n");
          jibing = 4;
        }
        //head to the right of the wind
        else {
          Serial.print("set tack to 5\n");
          jibing = 5;
        }
      }
      //heading to right, boat to left of wind
      else if (headingwind > 0) {
        //boat to left of wind
        if (boatwind < 0) {
          Serial.print("set tack to 6\n");
          jibing = 6;
        }
      }
      //heding to left, boat to right of wind
      else if (headingwind < 0) {
        if (boatwind > 0) {
          Serial.print("set tack to 7\n");
          jibing = 7;
        }
      }
    }
  }
  Serial.print("jibing: "); Serial.println(jibing);
  Serial1.print("jibing: "); Serial1.println(jibing);


  if (jibing == 0) {
    Serial.println("  head directly there"); Serial1.println("  head directly there");
  }
  else if (jibing == 1) {
    Serial.println("  turn back to get on heading"); Serial1.println("  turn back to get on heading");
  }
  else if (jibing == 2) {
    Serial.println("   head left of the wind upwind"); Serial1.println("   head left of the wind upwind");
  }
  else if (jibing == 3) {
    Serial.println("   head right of the wind upwind"); Serial1.println("   head right of the wind upwind");
  }
  else if (jibing == 4) {
    Serial.println("   head left of the wind downwind"); Serial1.println("   head left of the wind downwind");
  }
  else if (jibing == 5) {
    Serial.println("   head right of the wind downwind"); Serial1.println("   head right of the wind downwind");
  }
  else if (jibing == 6) {
    Serial.println("   jibe left"); Serial1.println("   jibe left");
  }
  else {
    Serial.println("   jibe right"); Serial1.println("   jibe right");
  }
  //jibing 0: head directly there
  //jibing 1: turn back to get on heading
  //jibing 2: head left of the wind upwind
  //jibing 3: head right of the wind upwind
  //jibing 4: head left of the wind downwind
  //jibing 5: head right of the wind downwind
  //jibing 6: jibe left
  //jibing 7: jibe right



  //check if we are way off our waypoint (possible extra input parameter) and if so then reset jibing
  //go through two points in a direction to simulate around buoy
  //bump in heading upwind (around tack point?)
  //turning wrong when for a sec when jibing
  //plot waypoints
  //optimize variables
  //heading at start depends on location of next next heading (eg if we want to go around it to the right or to the left)
  //start tacking before getting to the tacking point

  //DONE
  //plotting points
  Serial.print("windboat: "); Serial.println(windboat);

  //if we are going upwind or we need to keep going upwind
  //possibly redundant first boolean
  if ((fabsf(dirAngle) < optpolartop || fabsf(dirAngle) > 180 - optpolarbot) &&
      (jibing == 2 || jibing == 3 || jibing == 4 || jibing == 5)) {
    Serial.print("go at a tack: "); Serial.println(jibing); //printf("go at a tack: %f\n",jibing);
    Serial1.print("go at a tack: "); Serial1.println(jibing);
    // jibing=2;
    tackpointy = sensorData.lati;
    tackpointx = sensorData.longi;
    if (boatwind > 0) {
      sailAngle = windboat + angleofattack;
    }
    else {
      sailAngle = windboat - angleofattack;
    }
    if (jibing == 2) {
      // printf("up left of wind\n");
      if (fabsf(fabsf(boatwind) - optpolartop) > 30) {
        sailAngle = windboat;
      }
      //on correct heading
      if (fabsf(fabsf(boatwind) - optpolartop) < 5) {
        tailAngle = windboat;
      }
      //if the boat is to the right of optpolartop
      else if (boatwind + optpolartop < 0) {
        tailAngle = windboat - tailAngleOfAttack;
      }
      else {
        tailAngle = windboat + tailAngleOfAttack;
      }

    }
    //jibing==3
    else if (jibing == 3) {
      // printf("up right of wind\n");
      if (fabsf(fabsf(boatwind) - optpolartop) > 30) {
        sailAngle = windboat;
      }
      if (fabsf(fabsf(boatwind) - optpolartop) < 5) {
        tailAngle = windboat;
      }
      //if the boat is to the right of optpolartop
      else if (boatwind - optpolartop < 0) {
        tailAngle = windboat - tailAngleOfAttack;
      }
      else {
        tailAngle = windboat + tailAngleOfAttack;
      }
    }
    else if (jibing == 4) {
      // printf("down left of wind\n");
      if (fabsf(fabsf(boatwind) - (180 - optpolarbot)) > 30) {
        sailAngle = windboat;
      }
      if (fabsf(fabsf(boatwind) - (180 - optpolarbot)) < 5) {
        tailAngle = windboat;
      }
      //if the boat is to the left of optpolarbot - (-180+40)
      else if (boatwind + (180 - optpolarbot) < 0) {
        tailAngle = windboat - tailAngleOfAttack;
      }
      else {
        tailAngle = windboat + tailAngleOfAttack;
      }
    }
    else if (jibing == 5) {
      // printf("down right of wind\n");
      if (fabsf(fabsf(boatwind) - (180 - optpolarbot)) > 30) {
        sailAngle = windboat;
      }
      if (fabsf(fabsf(boatwind) - (180 - optpolarbot)) < 5) {
        tailAngle = windboat;
      }
      //if the boat is to the right of optpolarbot
      else if (boatwind - (180 - optpolarbot) < 0) {
        tailAngle = windboat + tailAngleOfAttack;
      }
      else {
        tailAngle = windboat - tailAngleOfAttack;
      }
    }
  }

  //stay upwind and head past the tacking point
  else if (fabsf(dirAngle) > optpolartop && (jibing == 2 || jibing == 3) && distance < jibedistance) {
    //else if(jibing ==3){

    // jibing=4;

    //might want to check if outside 30 degrees
    Serial.print("staying on heading past tacking point: "); Serial.println(jibing);
    Serial1.print("staying on heading past tacking point: "); Serial1.println(jibing);
    //printf("staying on heading past tacking point: %f\n",jibing);
    if (boatwind > 0) {
      sailAngle = windboat + angleofattack;
    }
    else {
      sailAngle = windboat - angleofattack;
    }
    if (jibing == 2) {
      // printf("up left of wind\n");
      //on correct heading
      if (fabsf(fabsf(boatwind) - optpolartop) < 5) {
        tailAngle = windboat;
      }
      //if the boat is to the right of optpolartop
      else if (boatwind + optpolartop < 0) {
        tailAngle = windboat - tailAngleOfAttack;
      }
      else {
        tailAngle = windboat + tailAngleOfAttack;
      }

    }
    //jibing==3
    else if (jibing == 3) {
      // printf("up right of wind\n");
      if (fabsf(fabsf(boatwind) - optpolartop) < 5) {
        tailAngle = windboat;
      }
      //if the boat is to the right of optpolartop
      else if (boatwind - optpolartop < 0) {
        tailAngle = windboat - tailAngleOfAttack;
      }
      else {
        tailAngle = windboat + tailAngleOfAttack;
      }
    }
  }
  //if we reach the point that we can start jibing
  // edit: changed from > to <
  else if ((distance >= jibedistance && (jibing == 2 || jibing == 3)) || (fabsf(dirAngle) < 180 - optpolarbot && ( jibing == 4 || jibing == 5))) {
    //else if(jibing==4){
    // printf("start jibing: %f, x: %f, y: %f\n",distance,tackpointx,tackpointy);
    Serial.print("start jibing: "); Serial.print(distance); Serial.print(" , x: "); Serial.print(tackpointx);
    Serial.print(", y: "); Serial.println(tackpointy);
    Serial1.print("start jibing: "); Serial1.print(distance); Serial1.print(" , x: "); Serial1.print(tackpointx);
    Serial1.print(", y: "); Serial1.println(tackpointy);
    sailAngle = windboat;
    if (differenceheading > 0) {
      tailAngle = windboat + tailAngleOfAttack;
    }
    else {
      tailAngle = windboat - tailAngleOfAttack;
    }
    if (jibing == 2 || jibing == 4) {
      jibing = 6;
    }
    else if (jibing == 3 || jibing == 5) {
      jibing = 7;
    }
  }

  //we are on our heading
  else if (fabsf(differenceheading) <  turnradius) {
    Serial.print("sail directly to waypoint\n"); Serial1.print("sail directly to waypoint\n");
    //else if(jibing==0){
    jibing = 0;
    //set tail into wind
    tailAngle = windboat;
    //generate lift, if wind is from right side vs left side angle of the attack is + or -
    if (boatwind > 0) {
      sailAngle = windboat + angleofattack;
    }
    else {
      sailAngle = windboat - angleofattack;
    }

  }
  else if (jibing == 6) {
    if (fabsf(differenceheading) < 30) {
      jibing = 0;
    }

    Serial.print("jibe left\n"); Serial1.print("jibe left\n");
    sailAngle = windboat;
    tailAngle = windboat + tailAngleOfAttack * 2;
  }
  else if (jibing == 7) {
    if (fabsf(differenceheading) < 30) {
      jibing = 0;
    }
    Serial.print("jibe right\n"); Serial1.print("jibe right\n");
    sailAngle = windboat;
    tailAngle = windboat - tailAngleOfAttack * 2;
  }
  else {
    Serial.print("general turn code\n"); Serial1.print("general turn code\n");
    //else if(jibing==1){
    //if the heading is to the right of the wind
    if (headingwind > 0) {
      // printf("heading to the right\n");
      //if the boat is to the right of the wind
      if (boatwind > 0) {
        //forward lift
        //if we are way off course, set sail into wind so we don't drift towards wrong heading
        if (fabsf(differenceheading) > 30) {
          sailAngle = windboat;
          if (differenceheading > 0) {
            tailAngle = windboat + tailAngleOfAttack * 2;
          }
          //if the heading is to the right of the boat
          else {
            tailAngle = windboat - tailAngleOfAttack * 2;
          }
        }
        else {
          //succesfully excecuted jibe
          jibing = 1;
          tackpointx = 0;
          tackpointy = 0;
          sailAngle = windboat + angleofattack;

          //if the heading is to the left of the boat
          if (differenceheading > 0) {
            tailAngle = windboat + tailAngleOfAttack;
          }
          //if the heading is to the right of the boat
          else {
            tailAngle = windboat - tailAngleOfAttack;
          }
        }
      }
      //if the boat is to the left of the wind
      //need to turn drastically
      else {
        sailAngle = windboat;
        tailAngle = windboat + tailAngleOfAttack * 2;
      }

    }
    //if the heading is to the left of the wind
    else {
      //if the boat is to the left of the wind
      if (boatwind < 0) {
        //forward lift
        //if we are way off course, set sail into wind so we don't drift towards wrong heading
        if (fabsf(differenceheading) > 30) {
          // printf("checking\n");
          sailAngle = windboat;
          if (differenceheading > 0) {
            tailAngle = windboat + tailAngleOfAttack * 2;
          }
          //if the heading is to the right of the boat
          else {
            tailAngle = windboat - tailAngleOfAttack * 2;
          }
        }
        else {

          //succesfully excecuted jibe
          jibing = 1;
          tackpointx = 0;
          tackpointy = 0;
          sailAngle = windboat - angleofattack;
          //if the heading is to the left of the boat
          if (differenceheading > 0) {
            tailAngle = windboat + tailAngleOfAttack;
          }
          //if the heading is to the right of the boat
          else {
            tailAngle = windboat - tailAngleOfAttack;
          }
        }
      }
      //if the boat is to the right of the wind
      //need to turn drastically
      else {
        sailAngle = windboat;
        tailAngle = windboat - tailAngleOfAttack * 2;
      }
    }


  }
  //  tailAngle=tailAngle*-1;
  //  sailAngle=sailAngle*-1;


  Serial.print("Sail angle (Raw):");   Serial.println(sailAngle);
  Serial.print("Tail angle (Raw):");   Serial.println(tailAngle);
  Serial1.print("Sail angle (Raw):");   Serial1.println(sailAngle);
  Serial1.print("Tail angle (Raw):");   Serial1.println(tailAngle);

  if (sailAngle < 0) {
    sailAngle += 360;
  }

  tailAngle = tailMap(sailAngle, tailAngle);
  sailAngle = sailMap(sailAngle);

  Serial.print("   Sail angle (Servo Command): ");   Serial.println(sailAngle);
  Serial.print("   Tail angle (Servo Command): ");   Serial.println(tailAngle);
  Serial1.print("   Sail angle (Servo Command): ");   Serial1.println(sailAngle);
  Serial1.print("   Tail angle (Servo Command): ");   Serial1.println(tailAngle);
  

  Serial.print("Next Waypoint #");
  Serial.print(wpNum);
  Serial.print(": ");
  Serial.print(wayPoint[wpNum].latitude);
  Serial.print(", ");
  Serial.println(wayPoint[wpNum].longitude);
  Serial.print("Next Heading: ");
  Serial.println(nextHeading);
  Serial.print("   Sail angle (servo): ");   Serial.println(sailAngle);
  Serial.print("   Tail angle (servo): ");   Serial.println(tailAngle);
  Serial.println();

  Serial1.print("Next Heading: ");
  Serial1.println(nextHeading);
  Serial1.print("   Sail angle (servo): ");   Serial1.println(sailAngle);
  Serial1.print("   Tail angle (servo): ");   Serial1.println(tailAngle);
  Serial1.println();
}

// adjusts servos
void nServos(void) {
  Serial.println("\n ----- Servos -----"); Serial1.println(" ----- Servos -----");

  // set tailAngle to sailAngle - tailAngle;
  sailServo.write(sailAngle);
  tailServo.write(tailAngle);
  
  Serial.print("tailAngle: ");    Serial1.print("tailAngle: ");
  Serial.print(tailAngle);        Serial1.print(tailAngle);
  Serial.print(", (actual) ");    Serial1.print(", (actual) ");
  Serial.println(tailServo.read() - tailMid);   Serial1.println(tailServo.read() - tailMid);
  Serial.print("sailAngle: ");    Serial1.print("sailAngle: ");
  Serial.print(sailAngle);        Serial1.print(sailAngle);
  Serial.print(", (actual) ");    Serial1.print(", (actual) ");
  Serial.println(sailServo.read() - sailMid);   Serial1.println(sailServo.read() - sailMid);
  Serial.println();               Serial1.println();
}

// calibrates sail servo with serial monitor input, enter desired angle, followed by period
void caltail(void) {
  String cmd;
  Serial.println("Enter desired tail angle, followed by period...");
  while (Serial.available() <= 0);
  char str = Serial.read();
  while (str != 46) {
    if (Serial.available()) {
      cmd += str;
      str = Serial.read();
    }
  }
  tailAngle = cmd.toFloat();
  Serial.print("Calibrating tail to ");
  Serial.println(tailAngle);
  Serial.println();
}

// calibrates tail servo with serial monitor input, enter desired angle, followed by period
void calSail(void) {
  String cmd;
  Serial.println("Enter desired sail angle, followed by period...");
  while (Serial.available() <= 0);
  char str = Serial.read();
  while (str != 46) {
    if (Serial.available()) {
      cmd += str;
      str = Serial.read();
    }
  }
  sailAngle = cmd.toFloat();
  Serial.print("Calibrating sail to ");
  Serial.println(sailAngle);
  Serial.println();
}


