//VERY PRIMITIVE VERSION
//Alec Dean



#include "mex.h"
//#include <stdio.h>
//#include <stdlib.h>
//#include "nShort.h"
// #include <Arduino.h>
#include <math.h>
// #include <Servo.h>
// #include "sensors.h"


typedef struct coordinate {
  double latitude; // float latitude
  double longitude; // float longitude
} coord_t;
int maxPossibleWaypoints=2;
int wpNum; //the current waypoint's number in the wayPoints array
int numWP; //total number of waypoints on current course
float detectionradius = 15; //how far away the boat marks a waypoint "reached"
coord_t wayPoints[3]; //the array containing the waypoints
float normr; //normal distance to the waypoint
float r[2]; //r[0]: Longitude difference b/w current position and waypoint,
            //r[1]: Lattitude difference b/w current position and waypoint
float w[2]; //w[0]: Cosine of the wind direction w.r.t North,
            //w[1]: Sine of the wind direction w.r.t North
float sailAngle;
float tailAngle;
float angleofattack=15;
float optpolartop=45;
float optpolarbot=40;
float windDir;
float boatDir;

/*---------Distance to Center Line------*/
// latitude is y, longitude is x
// float max_distance=100;
// coord_t center_start={1,2};
// coord_t center_end={10,20};

// float slope=(center_end.latitude - center_start.latitude)/(center_end.longitude - center_start.longitude);
// float intercept= center_start.latitude - slope * center_start.longitude;

// takes a coordinate and returns the distance from the coordinate to the center line
// float center_distance(coord_t position){
//   float top= fabs(slope*position.longitude+position.latitude+intercept);
//   float bot= sqrtf(slope*slope + 1);
//   return top/bot;

// }


float upRight(float b, float w) {
  printf("UP RIGHT\n\n");
  float offset = fabsf(w+optpolartop-b);
  printf("boatDir: %f\n windDir: %f\noffset: %f\n",b, w,offset);
  tailAngle=w-offset;
  return w-offset;
}

float rightTarget(float b, float w){
  // Serial.println("Right to target to the right");
  // Serial1.println("Right to target to the right");
  sailAngle=windDir+angleofattack;
  tailAngle=windDir;

  printf ("RIGHTS sail angle: %f\n",sailAngle);
  printf ("RIGHTT tail angle: %f\n",tailAngle);
  return w;
}

float leftTarget(float b, float w){

  printf("LEFT TARGET\n\n");
  // Serial.println("Right to target to the left");
  // Serial1.println("Right to target to the left");
  // sailAngle=windDir-angleofattack;
  return w;
}
// facing left, angle is above in the sector: w+offset
//   w+(|w-opttop-boatdir|)
float upLeft(float b, float w){
  printf("UP LEFT\n\n");
  // Serial.println("Left up left");
  // Serial1.println("Left up left");
  float offset = fabsf(windDir-optpolartop-b);
  return w+offset;
  // sailAngle=tailAngle-angleofattack;
}

// facing left, angle is below in the sector: w+offset
//   w+(|w+180-optboat-boatdir)
float downLeft(float b, float w){
  printf("DOWN RIGHT\n\n");
  // Serial.println("Left bottom left");
  // Serial1.println("Left bottom left");
  float offset = fabsf(windDir+180-optpolarbot-b);
  return w+offset;
  // sailAngle=tailAngle-angleofattack;
}
// facing right, angle is below in the sector: w-offset
//   w-(|w+180+optbot-boatdir|)
float downRight(float b, float w){
  printf("DOWN RIGHT\n\n");
  // Serial.println("Right bottom right");
  // Serial1.println("Right bottom right");
  float offset = fabsf(windDir+180+optpolarbot-b);
  return w-offset;
  // sailAngle=tailAngle+angleofattack;
}

float tackLeft(float b, float w, float angleWP){
  float offset=fabsf(w- optpolartop -b);
  printf ("offset: %f\n boatDir: %f\n", offset, b);
  return w+offset;
}

float tackRight(float b, float w, float angleWP){
  float offset=fabsf(w+ optpolartop - b);
  printf ("offset: %f\n boatDir: %f\n", offset, b);
  return w-offset;
}

void lightAllLEDs(){
  // digitalWrite(redLED1, HIGH);
  // digitalWrite(redLED2, HIGH);
  // digitalWrite(yellowLED, HIGH);
  // digitalWrite(blueLED, HIGH);
  // digitalWrite(greenLED, HIGH);
}

void lowAllLEDs(){
  // digitalWrite(redLED1, LOW);
  // digitalWrite(redLED2, LOW);
  // digitalWrite(yellowLED, LOW);
  // digitalWrite(blueLED, LOW);
  // digitalWrite(greenLED, HIGH);
}

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


// short term navigation algorithm, with the following inputs and outputs:
//   Inputs:
//      wayPoints - a double array representing the boat's destination points, as
//                  set by the long course code. Within array, values at even indices are
//                  latitude, and odd indices are longitude
//      sensorData - a size 2 double array containing boat's current latitude and longitude
//      windDir - a float between 0 and 360 that represents the current wind direction
//      boatDir - a float between 0 and 360 that represents the current boat direction
//      wpNum - index of the longitude value within wayPoints of the way point that
//              the boat is currently traveling towards
//      prevErr - difference between boat's last desired heading and its actually heading
//   Outputs:
//      angles - a size 4 float array containing rudder angle (float between 0 and 360) at index 0,
//               sail angle (float between 0 and 360) at index 1, the updated value of prevErr after this
//               calculation at index 3, and the updated value of wpNum at index 4
//
// Note: you should be initializing wpNum and prevErr as 0 within your simulation, and then
// updating them after each call to the short course algorithm
//
// Note 2: parts of the code that usually interact with Arduino sensors have been commented out, and replaced
// with code to interact with input parameters representing sensor information
void nShort(double *wayPoints, double *sensorData, float windDir, float boatDir, float wpNum, float waypointsize, float jibing, float tackpointx, float tackpointy, float *angles) {
  printf("\n\n");

  //Serial.println("Short Term Navigation");
  float nextHeading; float error;float command;
  mwSize alpha;  float n;
  float normr;  float v_T_R_max;
  float v_T_L_max;
  float v_T_R;  float v_T_L;
  float thetaBoat_R_max;
  float thetaBoat_L_max;
  float vB[2];  float r[2]; float w[2];
  float d1;  float d2;
  mwSize opt = 0;
  mwSize delalpha = 1;
  float PI = 3.14159265358979323846;
  float rudderAngle;

  //Calculates distance vector from boat to target, using latitude and longitude from sensor

//coming from north=0, E=90, S=180, W=270
  //dont need for arduino code
  windDir=360-(windDir-90);
    printf("windir: %f\n",windDir);
  //*****

  int waypointnumber=floor(wpNum);

  r[0] = wayPoints[waypointnumber] - sensorData[0];
  r[1] = wayPoints[waypointnumber+1] - sensorData[1];
  w[0] = cos((windDir)*(PI/180.0));
  w[1] = sin((windDir)*(PI/180.0));
  normr = sqrt(pow(r[0],2)+pow(r[1],2));


  float size =waypointsize;///sizeof(wayPoints[0]);

  printf("weird wind direction: %f\n",waypointnumber);

  // printf("checkingwaypoint normr: %f, wpNum: %f\n",normr,waypointnumber);

  float oldnormr=1000;

  float detectionradius=3;

  if((normr < detectionradius) && ((waypointnumber + 2) < size)){
    printf("reached waypoint\n");
    waypointnumber=waypointnumber+2;
    printf("new wpnum: %f\n\n\n\n", waypointnumber);

    //reset variables because we are close to a waypoint
    r[0] = wayPoints[waypointnumber] - sensorData[0];
    r[1] = wayPoints[waypointnumber+1] - sensorData[1];
    w[0] = cos((windDir)*(PI/180.0));
    w[1] = sin((windDir)*(PI/180.0));
    oldnormr=normr;
    normr = sqrt(pow(r[0],2)+pow(r[1],2));
    printf("new normr: %f\n",normr );
  }

  //optimal angle to go at if we cannot go directly to the waypoint
  //puts us on a tack or jibe
  //different values for top and bottom of polar plot
  float optpolartop=45;
  float optpolarbot=40;
  //bottom vs top
  //dir is the direction to the next waypoint from the boat
  printf("boat at: %f,%f\n",sensorData[0],sensorData[1]);
  //because we want the angle from the y axis (from the north) we take atan of adjacent over oppoosite
  float anglewaypoint=atan2(r[0],r[1])*360/(2*PI);
  //converts to 0-360
  anglewaypoint=(float)((int)anglewaypoint%360);
  anglewaypoint=anglewaypoint+360;
  anglewaypoint=(float)((int)anglewaypoint%360);




  float dirangle=anglewaypoint-windDir;
  //converts to 0-360
  dirangle=(float)((int)dirangle%360);
  dirangle=dirangle+360;
  dirangle=(float)((int)dirangle%360);

  boatDir=360-(boatDir-90);
  boatDir=(float)((int)boatDir%360);
  boatDir=boatDir+360;
  boatDir=(float)((int)boatDir%360);
  printf("boat orientation: %f\n",boatDir);
  //right=0, left =1
  int side;

//pseudocode for new sailing method---DOESNT WORK COMPLETELY--VERY PRIMITIVE VERSION

//angle to waypoint wrt wind

//only needed for simulator because boatdir in simulator is CCW wrt x axis
//boatDir=450-boatDir;
float angleofattack;
angleofattack = 15;

// wind wrt boat
float windboat;
windboat=windDir-boatDir;

float boat_wrt_wind=boatDir-windDir;
// boat_wrt_wind=360-(boat_wrt_wind-90);
boat_wrt_wind=(float)((int)boat_wrt_wind%360);
boat_wrt_wind=boat_wrt_wind+360;
boat_wrt_wind=(float)((int)boat_wrt_wind%360);

// tacking left: tell the boat it is facing the waypoint
// if (jibing==1){
//   if(boat_wrt_wind<=180){
//     tailAngle=turnLeft();
//     sailAngle=tailAngle-15;
//   }
//   else{
//     jibing=0;
//   }
// }

printf("BOAT WRT WIND: %f\n",boat_wrt_wind);
if (     jibing==1 && (boat_wrt_wind<=180+ optpolarbot || boat_wrt_wind>= 360-optpolartop)){
  printf("TACKING left\n");
  tailAngle=tackLeft(boatDir, windDir, anglewaypoint);
  sailAngle=tailAngle-15;
  // sailAngle=windDir;
  // tailAngle=windDir+30;
}
else if (jibing==2 && ( boat_wrt_wind>=180- optpolarbot && boat_wrt_wind<= optpolartop)){
  printf("TACKING right\n");
  tailAngle=tackRight(boatDir, windDir, anglewaypoint);
  sailAngle=tailAngle+15;
  // sailAngle=windDir;
  // tailAngle=windDir-30;
}
// tacking right
// else if (jibing==2){
//   if(boat_wrt_wind>=180){
//     boatDir=anglewaypoint;
//     boat_wrt_wind=boatDir-windDir;
//     // boat_wrt_wind=360-(boat_wrt_wind-90);
//     boat_wrt_wind=(float)((int)boat_wrt_wind%360);
//     boat_wrt_wind=boatDir+360;
//     boat_wrt_wind=(float)((int)boat_wrt_wind%360);
//   }
//   else{
//     jibing=0;
//   }
// }

// printf("Setting sail and tail");

  //dir is the direction to the next waypoint from the boat
  //because we want the angle from the y axis (from the north), we take atan of adjacent over oppoosite


  /*---------checking if past maximum tacking width------------*/
  // float distance_to_center = center_distance(currentPosition);
  // boat facing right and past line, must turn left
  // if (distance_to_center<max_distance){
  //   if (dirangle<180){
  //     Serial.println("Past line to the right, turning left");
  //     Serial1.println("Past line to the right, turning left");
  //     sailAngle=windDir-angleofattack;
  //     tailAngle=windDir;
  //   }
  //   else {
  //     Serial.println("Past line to the left, turning right");
  //     Serial1.println("Past line to the left, turning right");
  //     sailAngle=windDir+angleofattack;
  //     tailAngle=windDir;
  //   }
  // }
/* heading upwind angles
  general format:
    offset is the angle between the boat's actual orientation and desired orientation
    eg when if wind is at 0, opttop is at 45, and we boat is facing 20, offset is 25
    w is wind wrt north
    opttop and optboat are 45 or 40 eg
    boatdir is boat wrt north

  general: if boat is facing right, and cant sail directly to WP or turn to WP,
    tailangle= wind -offset
    facing left: tailangle=wind+offset
  facing right, angle is above in the sector: w-offset
    w-(|w+opttop - boatdir|)
  facing left, angle is below in the sector: w+offset
    w+(|w+180-optboat-boatdir)
  facing left, angle is above in the sector: w+offset
    w+(|w-opttop-boatdir|)
  facing right, angle is below in the sector: w-offset
    w-(|w+180+optbot-boatdir|)
*/


//  boat initially facing right
  else if (boat_wrt_wind<180) {
    jibing=0;
    printf ("facing right\n");
    //Up right
    if (dirangle<optpolartop && dirangle>=0){
      tailAngle=upRight(boatDir,windDir);
      sailAngle=tailAngle+15;
    }
    //Head directly to target to the right
    else if (dirangle>=optpolartop && dirangle<=180- optpolarbot){
      printf("DIRECT TO TARGET RIGHT\n");
      tailAngle=rightTarget(boatDir,windDir);
      sailAngle=tailAngle+15;
    }
    //Head directly to target to the left
    else if (dirangle>=optpolarbot + 180 && dirangle<=360 -optpolartop){
      // turning
      printf("DIRECT LEFT\n");
      // tailAngle=leftTarget(boatDir,windDir);
      // sailAngle=tailAngle-15;
      tailAngle=rightTarget(boatDir,windDir);
      sailAngle=tailAngle+15;
      jibing=1;
    }
    //Up left
    else if (dirangle>=360-optpolartop){
      tailAngle=upRight(boatDir,windDir);
      sailAngle=tailAngle+15;
    }
    //bottom left
    else if (dirangle <= 180 + optpolarbot && dirangle >= 180){
      tailAngle=downRight(boatDir,windDir);
      sailAngle=tailAngle+15;
    }
    //bottom right
    else {
      tailAngle=downRight(boatDir,windDir);
      sailAngle=tailAngle+15;
    }
  }
  //boat facing to left
  else{
    printf ("facing left\n");
    jibing=0;
    //Up right
    if (dirangle<optpolartop && dirangle>0){
      tailAngle=upLeft(boatDir,windDir);
      sailAngle=tailAngle-15;
    }
    //Head directly to target to the right
    else if (dirangle>optpolartop && dirangle<180- optpolarbot){
      // tailAngle=rightTarget(boatDir,windDir);
      // sailAngle=tailAngle+15;
      tailAngle=leftTarget(boatDir, windDir);
      sailAngle=tailAngle-15;
      jibing=2;
    }
    //Head directly to target to the left
    else if (dirangle>optpolarbot + 180 && dirangle<360 -optpolartop){
      tailAngle=leftTarget(boatDir,windDir);
      sailAngle=tailAngle-15;
    }
    //Up left
    else if (dirangle>360-optpolartop){
      tailAngle=upLeft(boatDir,windDir);
      sailAngle=tailAngle-15;
    }
    //bottom left
    else if (dirangle < 180 + optpolarbot && dirangle > 180){
      tailAngle=downLeft(boatDir,windDir);
      sailAngle=tailAngle-15;
    }
    //bottom right
    else {
      tailAngle=downLeft(boatDir,windDir);
      sailAngle=tailAngle-15;
    }
  }


  sailAngle=(float)((int)sailAngle%360);
  sailAngle=sailAngle+360;
  sailAngle=(float)((int)sailAngle%360);
  printf ("output sail angle: %f\n",sailAngle);
  printf ("output tail angle: %f\n",tailAngle);


// ////////////////////////////////////////////////








  sailAngle=360-(sailAngle-90);
  tailAngle=360-(tailAngle-90);



  angles[0] = tailAngle;
  angles[1] = sailAngle;
  angles[2] = jibing;
  angles[3] = jibing;
  angles[4] = tackpointx;
  angles[5] = tackpointy;
  angles[6] = waypointnumber;
}

// The gateway function
// This functions assists in creating a mex file. You run it by
// calling 'mex nShort.c' in your command window, which will create the
// mex file and thus allow you to call 'nShort(wayPoints,sensorData,windDir,boatDir,wpNum,prevErr)
// within any other MATLAB files in the same directory as the mex file
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{
    float wpNum;
    double *wPs;
    double *sD;
    float wDir;
    float bDir;
    float pError;
    float waypointsize;
    float tackpointx;
    float tackpointy;
    float *angles;

    //double multiplier;              /* input scalar */
    //double *inMatrix;               /* 1xN input matrix */
    //size_t ncols;                   /* size of matrix */
    //double *outMatrix;              /* output matrix */

    /* check for proper number of arguments */
    if(nrhs!=9) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nrhs","8 inputs required.");
    }
    if(nlhs!=1) {
        if(nlhs==2){mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nlhs","One output required. There were 2");}
        else if(nlhs==0){mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nlhs","One output required. There were 0");}
        else{mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nlhs","One output required.");}
    }

    /* make sure the first input argument is type float */
    if( mxIsComplex(prhs[0])) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:notDouble","Input matrix must be type float.");
    }

    /* check that number of rows in first input argument is 1 */
    if(mxGetM(prhs[0])!=1) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:notRowVector","Input must be a row vector.");
    }

    /* make sure the second input argument is type float */
    if( mxIsComplex(prhs[1])) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:notDouble","Input matrix must be type float.");
    }

    /* check that number of rows in second input argument is 1 */
    if(mxGetM(prhs[1])!=1) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:notRowVector","Input must be a row vector.");
    }

    /* make sure the third input argument is scalar */
    if( mxIsComplex(prhs[2]) ||
         mxGetNumberOfElements(prhs[2])!=1 ) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:notScalar","Input multiplier must be a scalar.");
    }

    /* make sure the fourth input argument is scalar */
    if( mxIsComplex(prhs[3]) ||
         mxGetNumberOfElements(prhs[3])!=1 ) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:notScalar","Input multiplier must be a scalar.");
    }

    /* make sure the fifth input argument is scalar */
    if( mxIsComplex(prhs[4]) ||
         mxGetNumberOfElements(prhs[4])!=1 ) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:notScalar","Input multiplier must be a scalar.");
    }

    /* make sure the sixth input argument is scalar */
    if( mxIsComplex(prhs[5]) ||
         mxGetNumberOfElements(prhs[5])!=1 ) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:notScalar","Input multiplier must be a scalar.");
    }

    /* get the value of the scalar inputs  */
    wDir = (float) mxGetScalar(prhs[2]);
    bDir = (float) mxGetScalar(prhs[3]);
    pError = (float) mxGetScalar(prhs[4]);
    wpNum = (float) mxGetScalar(prhs[5]);
    waypointsize= (float) mxGetScalar(prhs[6]);
    tackpointx = (float) mxGetScalar(prhs[7]);
    tackpointy = (float) mxGetScalar(prhs[8]);

    /* create a pointer to the real data in the input matrices  */
    wPs = mxGetPr(prhs[0]);
    sD = mxGetPr(prhs[1]);
    //sD = (float*) mxGetPr(prhs[1]);

    /* create the output matrix */
    plhs[0] = mxCreateNumericMatrix(1, 7, mxSINGLE_CLASS, mxREAL);
    //plhs[0] = mxCreateDoubleMatrix(1,(mwSize)ncols,mxREAL);

    /* get a pointer to the real data in the output matrix */
    angles = (float *) mxGetData(plhs[0]);

    /* call the computational routine */
    nShort(wPs,sD,wDir,bDir,pError,wpNum, waypointsize, tackpointx, tackpointy, angles);
}
