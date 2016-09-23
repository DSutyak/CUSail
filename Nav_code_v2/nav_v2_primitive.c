//VERY PRIMITIVE VERSION
//Alec Dean



#include "mex.h"
//#include <stdio.h>
//#include <stdlib.h>
//#include "nShort.h"
//#include <Arduino.h>
#include <math.h>

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
  
  float w[2];
  float d1;
  float d2;
  mwSize opt = 0;
  mwSize delalpha = 1;
  float PI = 3.14159265358979323846;
  float rudderAngle;
  float sailAngle;
  float tailangle;

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

    printf("Setting sail and tail");
    if (dirangle<optpolartop && dirangle>0){
      sailAngle=optpolartop -angleofattack;
      tailangle=optpolartop;
    }
    //head directly to target to the right
    else if (dirangle>optpolartop && dirangle<180- optpolarbot){
      sailAngle=dirangle-angleofattack;
      tailangle=dirangle;
    }

    else if (fabsf(dirangle)>optpolartop && fabsf(dirangle)<180 -optpolarbot){
      sailAngle=dirangle-angleofattack;
      tailangle=dirangle;
    }
    else if (dirangle>360-optpolartop){
      sailAngle=optpolartop +angleofattack;
      tailangle=optpolartop;
    }
    else if (dirangle<360-optpolartop){
      sailAngle=dirangle +angleofattack;
      tailangle=dirangle;
    }


  angles[0] = tailangle;
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
