#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include "sensors.h"
#include "navigation.h"

Servo tailServo; // tail ?
Servo sailServo;
float prevErr;
//float tailAngle;
//float sailAngle;
//coord_t* wayPoints;
coord_t wayPoints[MAX_WAYPOINTS];
int wpNum;
// ------------------------
int numWp;
float nextHeading;
float error;
float command;
int alpha; // mwSize
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
int opt = 0; // mwSize
int delalpha = 1; // mwSize
//float PI = 3.14159265358979323846;
// float tailAngle;
float sailAngle;
float tailAngle;

float optpolartop=45;
float optpolarbot=40;
float angleofattack= 15; // 10; //15;
int side;


int LEDcount = 0;

// polar plot gives expected speed of boat at a given heading with respect to the wind
// 0 degree heading represents sailing directly into the windpre
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

// initializes servos
void initServos(void) {
  tailServo.attach(tailServoPin);
  tailServo.write(rudderMid);
  sailServo.attach(sailServoPin);
  sailServo.write(sailMid);
  tailAngle = 0;
  sailAngle = 0;
}

// initializes navigation algorithm
void initNavigation(void) {
  //wayPoints = {0};
  prevErr = 0;
  wpNum = 0;
  pinMode(LED, OUTPUT);
}

// sets waypoints
// void setWaypoints(coord_t wPoint[]) {
void setWaypoints(void) {
  //wayPoints=wPoint;
  coord_t asdf {5,7};
  coord_t p1 = {42.444544, -76.483399}, p2 = {42.444821, -76.483566}, p3 = {100,100}, 
          olin = {42.445457, -76.484322}, hollister = {42.444259, -76.484435}, 
          engQuadStairs = {42.444287, -76.484483}, p0 = {0.0,0.0},
          engQuadX = {42.444609,-76.483494}, otherStairs{42.444792,-76.483228},
          sundial = {42.444895,-76.483622}, middleQuad = {42.444374,-76.483340},
          topDuffStairs = {42.444301,-76.482892}, ptest = {-30, 0},
          bottomStairs = {42.444273, -76.483084}, 
          midQuad2 = {42.444499, -76.483677};
//  p1.latitude = 42.444544;  p1.longitude = -76.483399;
//  p2.latitude = 42.444821;  p2.longitude = -76.483566;
//  p3.latitude = 100; //43;  p3.longitude = 100; //-77;
//  olin.latitude = 42.445457;          olin.longitude = -76.484322;
//  hollister.latitude = 42.444343;     hollister.longitude = -76.484483;
//  engQuadStairs.latitude = 42.444287; engQuadStairs.longitude = -76.483222;
    wayPoints[0] =   {42.444278, -76.483267};
                  // bottomStairs; //engQuadX; // topDuffStairs; // {42.443870,-76.481970}; //otherStairs; 
                    // topDuffStairs; //ptest; // {42.444280,-76.482902}; //p0; //engQuadStairs;
//  wayPoints[1] = engQuadX;
//  wayPoints[2] = otherStairs;
//  wayPoints[3] = sundial;
//  wayPoints[4] = middleQuad;
//  wayPoints[5] = topDuffStairs;
//  wayPoints[6] = p3;
    wayPoints[1] = midQuad2; // engQuadX; //engQuadX; // otherStairs;
    wayPoints[2] = {42.444278, -76.483267}; // bottomStairs; //engQuadX;
//    wayPoints[3] = middleQuad;
//    wayPoints[4] = topDuffStairs;
    numWp = 3; // 5
    //numWp = sizeof(wayPoints)/sizeof(wayPoints[0]);
    Serial.print("number of waypoints: "); Serial.println(numWp);
  Serial.println("First coordinate: ");
  Serial.print("latitude: "); Serial.println(wayPoints[0].longitude,6);
  Serial.print("longitude: "); Serial.println(wayPoints[0].latitude,6);
  Serial.println("\nSecond coordinate: ");
  Serial.print("latitude: "); Serial.println(wayPoints[1].longitude,6);
  Serial.print("longitude: "); Serial.println(wayPoints[1].latitude,6);
  Serial.println("\nThird coordinate: ");
  Serial.print("latitude: "); Serial.println(wayPoints[2].longitude,6);
  Serial.print("longitude: "); Serial.println(wayPoints[2].latitude,6);
  Serial.println("\npath: quad intersection to stairs to sundial to middle quad to duffield stairs");
}

void nShort(void) {
  Serial.println("\n --- Short Term Navigation ---");
  //printf("printing test: %f\n", sensorData.windDir);
  Serial.print("Current location:    Latitude: "); Serial.print(sensorData.lati,6); 
  Serial.print("  Longitude: "); Serial.println(sensorData.longi,6);
  Serial.print("Next waypoint #"); Serial.print(wpNum); 
  Serial.print(":    Latitude: "); Serial.print(wayPoints[wpNum].latitude,6); 
  Serial.print("  Longitude: "); Serial.println(wayPoints[wpNum].longitude,6);
  
    
//  float polarPlot [361] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
//                        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
//                        0,0,0,0,0.1712,0.17325,0.17742,0.18336,0.19064,0.19882,0.20757,0.21661,
//                        0.22575,0.23487,0.24389,0.25275,0.26143,0.26991,0.27818,0.28623,0.29407,0.3017,0.30912,0.31634,
//                        0.32337,0.33021,0.33687,0.34335,0.34966,0.35581,0.36179,0.36761,0.37329,0.37882,0.3842,0.38944,
//                        0.39455,0.39952,0.40437,0.40909,0.41368,0.41815,0.42251,0.42675,0.43087,0.43488,0.43878,0.44257,
//                        0.44626,0.44984,0.45332,0.4567,0.45998,0.46315,0.46624,0.46922,0.47211,0.47491,0.47761,0.48023,
//                        0.48275,0.48518,0.48753,0.48979,0.49196,0.49405,0.49605,0.49797,0.4998,0.50155,0.50322,0.50481,
//                        0.50632,0.50774,0.50909,0.51036,0.51155,0.51267,0.5137,0.51466,0.51555,0.51636,0.51709,0.51775,
//                        0.51833,0.51884,0.51927,0.51964,0.51992,0.52014,0.52028,0.52035,0.52035,0.52028,0.52013,0.51991,
//                        0.51962,0.51926,0.51883,0.51833,0.51775,0.51711,0.51639,0.5156,0.51474,0.51381,0.51281,0.51173,
//                        0.51059,0.50937,0.50808,0.50672,0.50529,0.50378,0.5022,0.50055,0.49882,0.49702,0.49514,0.49319,
//                        0.49117,0.48907,0.48689,0.48463,0.4823,0.47989,0.4774,0.47483,0.47218,0.46945,0.46663,0.46373,
//                        0.46075,0.46373,0.46663,0.46945,0.47218,0.47483,0.4774,0.47989,0.4823,0.48463,0.48689,0.48907,
//                        0.49117,0.49319,0.49514,0.49702,0.49882,0.50055,0.5022,0.50378,0.50529,0.50672,0.50808,0.50937,
//                        0.51059,0.51173,0.51281,0.51381,0.51474,0.5156,0.51639,0.51711,0.51775,0.51833,0.51883,0.51926,
//                        0.51962,0.51991,0.52013,0.52028,0.52035,0.52035,0.52028,0.52014,0.51992,0.51964,0.51927,0.51884,
//                        0.51833,0.51775,0.51709,0.51636,0.51555,0.51466,0.5137,0.51267,0.51155,0.51036,0.50909,0.50774,
//                        0.50632,0.50481,0.50322,0.50155,0.4998,0.49797,0.49605,0.49405,0.49196,0.48979,0.48753,0.48518,
//                        0.48275,0.48023,0.47761,0.47491,0.47211,0.46922,0.46624,0.46315,0.45998,0.4567,0.45332,0.44984,
//                        0.44626,0.44257,0.43878,0.43488,0.43087,0.42675,0.42251,0.41815,0.41368,0.40909,0.40437,0.39952,
//                        0.39455,0.38944,0.3842,0.37882,0.37329,0.36761,0.36179,0.35581,0.34966,0.34335,0.33687,0.33021,
//                        0.32337,0.31634,0.30912,0.3017,0.29407,0.28623,0.27818,0.26991,0.26143,0.25275,0.24389,0.23487,
//                        0.22575,0.21661,0.20757,0.19882,0.19064,0.18336,0.17742,0.17325,0.1712,0,0,0,
//                        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
//                        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  // outdated polar plot
  
  //Serial.println("Short Term Navigation");


  //Calculates distance vector from boat to target, using latitude and longitude from sensor

//coming from north=0, E=90, S=180, W=270
  //sensorData.windDir=360-(sensorData.windDir-90);

  // printf("windidr: %f\n",sensorData.windDir);

  int windDirN = ((int)(sensorData.windDir + sensorData.boatDir))%360;

//  r[0] = wayPoints[wpNum] - sensorData[0];
//  r[1] = wayPoints[wpNum+1] - sensorData[1];
  r[0] = wayPoints[wpNum].latitude - sensorData.lati;
  r[1] = wayPoints[wpNum].longitude - sensorData.longi;
  w[0] = cos((windDirN)*(PI/180.0));
  w[1] = sin((windDirN)*(PI/180.0));
  //r[0] = wayPoints[wpNum].latitude - sensorData.lati;
  //r[1] = wayPoints[wpNum].longitude - sensorData.longi;
  normr = sqrt(pow(r[0],2)+pow(r[1],2));

  Serial.print("Distance to waypoint: "); Serial.println(normr,6);
  Serial.print("    difference in latitude: "); Serial.println(r[0],6);
  Serial.print("    difference in longitude: "); Serial.println(r[1],6);
  
  //digitalWrite(LED, HIGH); // turn on LED

//  if (LEDcount > 0) {
//    LEDcount--;
//  }
//  else {
    digitalWrite(LED, LOW); // turn off LED
//  }

  //If the boat has reached its waypoint, starts calculating short term navigation towards next waypoint
  //But only if there is another waypoint to look at
  //if (normr < wpErr) {
  // int size = 60;    // mwSize
  if (normr < wpErr) {
  //if (normr < 1 && (wpNum + 2) < size) {
    // printf("reached waypoint");
    // digitalWrite(13, HIGH); // turn on LED
    Serial.print("\n   ----> WAYPOINT #"); Serial.print(wpNum); Serial.println(" REACHED\n");
    digitalWrite(LED, HIGH); // turn on LED
    LEDcount = 5;
    // wpNum = wpNum + 2;
    if ((wpNum + 1) < numWp) {
       wpNum = wpNum + 1;
  //    r[0] = wayPoints[wpNum] - sensorData[0];
  //    r[1] = wayPoints[wpNum+1] - sensorData[1];
//      r[0] = wayPoints[wpNum].latitude - sensorData.lati;
//      r[1] = wayPoints[wpNum].longitude - sensorData.longi;
      r[0] = sensorData.lati - wayPoints[wpNum].latitude;
      r[1] = sensorData.longi - wayPoints[wpNum].longitude;
      normr = sqrt(pow(r[0],2)+pow(r[1],2));
      Serial.print("Distance to waypoint: "); Serial.println(normr,6);
      Serial.print("    difference in latitude: "); Serial.println(r[0],6);
      Serial.print("    difference in longitude: "); Serial.println(r[1],6);
    }
    else {
      Serial.println("no more waypoints\n");
    }
  }
  Serial.println("\nfinding next heading:");
  Serial.print("windDir wrt North: "); Serial.println(windDirN);
//   mwSize deg = (mwSize) floor(atan(r[1]/r[0])*(180.0/PI)) - windDir;
//   if(deg < 0) { deg = deg + 360; }
//   
//   if (polarPlot[(deg)] > 0 && floor(deg + windDir - boatDir) < -40000){
//       deg = (mwSize) floor(deg + windDir);
//       if(deg > 360){ deg = deg - 360; }
//       nextHeading = deg;
//   }


  /*
      Next heading code start

  */
  //optimal angle to go at if we cannot go directly to the waypoint
  //puts us on a tack or jibe
  //different values for top and bottom of polar plot
//  float optpolartop=45;
//  float optpolarbot=40;
//  float angleofattack=15;
  //bottom vs top
  //dir is the direction to the next waypoint from the boat
  //printf("waypoint at: %f,%f\n",wayPoints[0],wayPoints[1]);
  //printf("boat at: %f,%f\n",sensorData[0],sensorData[1]);
  //because we want the angle from the y axis (from the north) we take atan of adjacent over oppoosite
  //float anglewaypoint=atan(r[0]/r[1]); //*360/(2*PI);
  //float anglewaypoint = atan2(r[0],r[1])*(180/PI);
  float anglewaypoint = atan2(r[1],r[0])*(180/PI);
  
  //converts to 0-360
  //anglewaypoint=(float)((int)anglewaypoint%360);
  if (anglewaypoint < 0) {
    anglewaypoint=anglewaypoint+360;
  }
  //anglewaypoint=(float)((int)anglewaypoint%360);
  // printf("angle north to waypoint: %f\n",anglewaypoint);
  //Serial.print("Angle to waypoint using atan "); Serial.println(atan(r[0]/r[1]));
  Serial.print("Angle to waypoint using atan2: "); Serial.println(anglewaypoint);

  //andgle from the wind to the waypoint vector. this is used to determine if we can sail to the next heading
  //or if we have to tack/jibe
  float dirangle=anglewaypoint - windDirN;
  //converts to 0-360
  dirangle=(float)((int)dirangle%360);
  dirangle=dirangle+360;
  dirangle=(float)((int)dirangle%360);
  // printf("dirangle: %f\n",dirangle);
  Serial.print("Angle from wind to waypoint: "); Serial.println(dirangle);
 
//  sensorData.boatDir = 360 - (sensorData.boatDir-90);
//  sensorData.boatDir=(float)((int)sensorData.boatDir%360);
//  sensorData.boatDir=sensorData.boatDir+360;
//  sensorData.boatDir=(float)((int)sensorData.boatDir%360);

   // printf("boat orientation: %f\n",boatDir);
    //right=0, left =1
//     int side;

  //we have to port tack into the wind
  //right up
  if(dirangle<optpolartop){
    // printf("right up port tack");
    Serial.println("right up / port tack");
    nextHeading=optpolartop;
   // side=0;
//    // sailAngle=-1*(nextHeading-windDir-angleofattack);
//    sailAngle=180-nextHeading-angleofattack;
//    sailAngle=sensorData.windDir+sensorData.boatDir-angleofattack;
  }
  //we can sail directly to the target
  //right side
  else if(dirangle<(180-optpolarbot)){
    Serial.println("sail directly to target; right side"); // printf("right side");
    nextHeading=dirangle;
   // side=0;
    // sailAngle=-1*(nextHeading-windDir-angleofattack);
//    sailAngle=180-nextHeading-angleofattack;
//    sailAngle=sensorData.windDir+sensorData.boatDir-angleofattack;
  }
  //we have to port jibe
  //right down
  else if(dirangle<(180)){
    Serial.println("right down / port jibe"); // printf("right down port jibe");
    nextHeading=180-optpolarbot-angleofattack;
//    side=0;
//    // sailAngle=-1*(nextHeading-windDir-angleofattack);
//    sailAngle=180-nextHeading-angleofattack;
//    sailAngle=sensorData.windDir+sensorData.boatDir-angleofattack;
  }
  //we have to starboard jibe
  //left down
  else if(dirangle<(180+optpolarbot)){
    Serial.println("left down / starboard jibe"); //     printf("left down starboard jibe");
    nextHeading=180+optpolarbot;
//    side=1;
//    // sailAngle=(nextHeading-windDir+angleofattack);
//    sailAngle=180-nextHeading+angleofattack;
//    sailAngle=sensorData.windDir+sensorData.boatDir+angleofattack;
  }
  //can sail directly to the target
  //left side
  else if(dirangle<(360-optpolartop)){
    Serial.println("sail directly to target; left side");    // printf("left side");
    nextHeading=dirangle;
//    side=1;
//    // sailAngle=(nextHeading-windDir+angleofattack);
//    sailAngle = 180 - nextHeading + angleofattack;
//    sailAngle = sensorData.windDir + sensorData.boatDir + angleofattack;
  }
  //starboard tack
  //left up
  else{
    Serial.println("left up / starboard tack");   //     printf("left up starboard tack");
    nextHeading=360-optpolartop;
//    side=1;
//    // sailAngle=360-(nextHeading-windDir+angleofattack);
//    sailAngle = 180 - nextHeading + angleofattack;
//    sailAngle = sensorData.windDir + sensorData.boatDir + angleofattack;
  }

  // printf("next heading wrt wind:%f\n",nextHeading);
  Serial.print("next heading wrt wind: ");  Serial.println(nextHeading);
  //convert back to wrt North
  nextHeading=nextHeading + windDirN;
  //converts to 0-360
  nextHeading=(float)((int)nextHeading%360);
  nextHeading=nextHeading+360;
  nextHeading=(float)((int)nextHeading%360);

  // printf("next heading:%f\n",nextHeading);
   Serial.print("next heading: ");  Serial.println(nextHeading);

  /*next heading code end
  */
  //sail code
  // if(side==0){

  //   sailAngle=nextHeading-windDir-angleofattack;
  // }
  // else{

  //   sailAngle=nextHeading-windDir+angleofattack;
  // }


  //need to turn right
  // printf("change heaing: %f\n",boatDir-nextHeading);
//  if((sensorData.boatDir-nextHeading)>10){
//    tailAngle = sensorData.windDir + sensorData.boatDir + 15;
//  }
//  else if((sensorData.boatDir-nextHeading)<-10){
//    tailAngle = sensorData.windDir+sensorData.boatDir-15;
//  }
//  else{
//    tailAngle = sensorData.windDir + sensorData.boatDir;
//  }
// -----------------------------------------------------

  float turnradius=5;
  //greater means turn left
  float differenceheading=sensorData.boatDir-nextHeading;
  float boatwind=sensorData.boatDir-windDirN;
  float tailangleofattack=15;
  float headingwind=nextHeading-windDirN;
  if(headingwind>180){
    headingwind-=360;
  }
  if(headingwind<-180){
    headingwind+=360;
  }
  if(boatwind>180){
    boatwind-=360;
  }
  if(boatwind<-180){
    boatwind+=360;
  }
  //float angleofattack=15;
  // float windside;
  // if(windside<0){
  //   windside=windside+360;
  // }
  // if(windside<180){
  //   windside=0;
  // }
  // else{
  //   windside=1;
  // }
  //convert to counter clockwise direction
  //sail angle directly into the wind
  float windboat=windDirN-sensorData.boatDir;
  //convert to 360
  // if(windboat<0){
  //   windside=windboat+360;
  // }
  // // wind coming from right side of boat
  // if(windboat>0){
  //   windside=0;
  // }
  // //wind coming from left side of boat
  // else{
  //   windside=1;
  // }
  // float windheading=windDir-nextHeading;
  
  //if we are trying to go directly upwind or downwind
  //JIBE CODE
  //TODOneed to make sure boat doesnt go off course
  // if(dirangle>180){
  //   dirangle-=360;
  // }
  // float jibeangle=10;
  // if(fabsf(boatwind-optpolartop)<10 && fabsf(dirangle)<optpolartop+jibeangle){
  // if(fabsf(dirangle)<(optpolartop+jibeangle) && fabsf(boatwind)<90){
  //   printf("going upwind\n\n\n");
  //   if(boatwind>0){
  //     sailAngle=windboat+angleofattack;
  //   }
  //   else{
  //     sailAngle=windboat-angleofattack;
  //   }
  //   tailangle=windboat;
  //   if (fabsf(fabsf(boatwind)-optpolartop)>  turnradius){
  //     //set tail into wind
  //     //generate lift, if wind is from right side vs left side angle of the attack is + or -
  //     if(fabsf(boatwind)>optpolartop){
  //       tailangle=windboat+tailangleofattack/4;
  //     }
  //     else {
  //       printf("\n\n\nturn away from the wind\n");
  //       tailangle=windboat-tailangleofattack/2;
  //     }
  //   }
  // }
  //we are on our heading
  //put in else if if input jibe code
  Serial.print("windboat: "); Serial.println(windboat);
  Serial.print("headingwind: "); Serial.println(headingwind);
  //Serial.print("boatwind: "); Serial.println(boatwind);
  if (fabsf(differenceheading)<  turnradius){
  // if(false){
      //set tail into wind
      Serial.println("set tail into wind\ntailangle=windboat");
      tailAngle=windboat;
      //generate lift, if wind is from right side vs left side angle of the attack is + or -
      if(boatwind>0){
        sailAngle=windboat+angleofattack;
        Serial.println("sail angle = windboat + angleofattack");
      }
      else {
        sailAngle=windboat-angleofattack;
        Serial.println("sail angle = windboat - angleofattack");
      }
  }
  //not in range of heading so set sail into wind and tail in direction to turn
  else{
      //if the heading is to the right of the wind
      if(headingwind>0){
        //printf("heading to the right\n");
        Serial.println("heading to the right");
          //if the boat is to the right of the wind
          if(boatwind>0){
              //forward lift
              Serial.println("sailAngle=windboat+angleofattack");
              sailAngle=windboat+angleofattack;
              //if the heading is to the left of the boat
              if(differenceheading>0){
                tailAngle=windboat+tailangleofattack;
                Serial.println("tailAngle = windboat + tailangleofattack");
              }
              //if the heading is to the right of the boat
              else{
                tailAngle=windboat-tailangleofattack;
                Serial.println("tailAngle = windboat - tailangleofattack");
              }
          }
          //if the boat is to the left of the wind
          //need to turn drastically
          else{
            Serial.println("need to turn drastically\nsailangle=windboat\ntailangle=windboat+tailangleofattack*2");
            sailAngle=windboat;// + angleofattack/5;
            tailAngle=windboat+tailangleofattack*2;
          }
      }
      //if the heading is to the left of the wind
      else{
          //if the boat is to the left of the wind
          if(boatwind<0){
              //forward lift
              Serial.println("sailAngle=windboat - angleofattack");
              
              sailAngle=windboat-angleofattack;
              //if the heading is to the left of the boat
              if(differenceheading>0){
                tailAngle=windboat+tailangleofattack;
                Serial.println("tailAngle=windboat + tailangleofattack");
              }
              //if the heading is to the right of the boat
              else{
                tailAngle=windboat-tailangleofattack;
                Serial.println("tailAngle=windboat - tailangleofattack");
              }
          }
          //if the boat is to the right of the wind
          //need to turn drastically
          else{
            Serial.println("need to turn drastically\nsailangle=windboat\ntailangle=windboat-tailangleofattack*2");
            sailAngle=windboat;//-angleofattack/5;
            tailAngle=windboat-tailangleofattack*2;
          }
    }
  }

  if (sailAngle < 0) {
    sailAngle += 360;
  }
  if (tailAngle < 0) {
    tailAngle += 360;
  }

  if (sailAngle > 360) {
    sailAngle -= 360;
  }
  if (tailAngle > 360) {
    tailAngle -= 360;
  }
  
  // sail and tail angles are 0 to 360 wrt boat
  
  Serial.print("Next Waypoint #");
  Serial.print(wpNum);
  Serial.print(": ");
  Serial.print(wayPoints[wpNum].latitude);
  Serial.print(", ");
  Serial.println(wayPoints[wpNum].longitude);
  Serial.print("Next Heading: ");
  Serial.println(nextHeading);
  Serial.print("   Sail angle: ");   Serial.println(sailAngle);
  Serial.print("   Tail angle: ");   Serial.println(tailAngle);
  Serial.println();
}

// adjusts servos
void nServos(void) {
//  sailAngle = 120;
//  tailAngle = 57;

  // sail angle is with respect to bow of boat
  // eventually, tail angle will need to be set using sail angle
  
  Serial.println("\n----- Servos -----");
//  tailServo.write(rudderMid+tailAngle);
//  sailServo.write(sailMid+sailAngle); 
  tailServo.write(tailAngle);
  sailServo.write(sailAngle); 
  Serial.print("tailAngle: ");
  Serial.print(tailAngle);
  Serial.print(", (actual) ");
  // Serial.print(tailServo.read()-rudderMid);
  Serial.print(tailServo.read());
  Serial.print(", sailAngle: ");
  Serial.print(sailAngle);
  Serial.print(", (actual) ");
  // Serial.println(sailServo.read()-sailMid);
  Serial.println(sailServo.read());
  Serial.println();
}

// calibrates sail servo with serial monitor input, enter desired angle, followed by period
void calRudder(void) {
  String cmd;
  Serial.println("Enter desired rudder angle, followed by period...");
  while(Serial.available() <= 0);
  char str = Serial.read();
  while(str != 46) {
    if(Serial.available()) {
      cmd += str;
      str = Serial.read();
    }
  }
  tailAngle = cmd.toFloat();
  Serial.print("Calibrating rudder to ");
  Serial.println(tailAngle);
  Serial.println();
}

// calibrates rudder servo with serial monitor input, enter desired angle, followed by period
void calSail(void) {
  String cmd;
  Serial.println("Enter desired sail angle, followed by period...");
  while(Serial.available() <= 0);
  char str = Serial.read();
  while(str != 46) {
    if(Serial.available()) {
      cmd += str;
      str = Serial.read();
    }
  }
  sailAngle = cmd.toFloat();
  Serial.print("Calibrating sail to ");
  Serial.println(sailAngle);
  Serial.println();
}


