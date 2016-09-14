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
void nShort(double *wayPoints, double *sensorData, float windDir, float boatDir, mwSize wpNum, float prevErr, float *angles) {
  
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
  
  float d1;
  float d2;
  mwSize opt = 0;
  mwSize delalpha = 1;
  float PI = 3.14159265358979323846;
  float rudderAngle;
  float sailAngle;

  //Calculates distance vector from boat to target, using latitude and longitude from sensor
  r[0] = wayPoints[wpNum] - sensorData[0];
  r[1] = wayPoints[wpNum+1] - sensorData[1];
  //r[0] = wayPoints[wpNum].latitude - sensorData.lati;
  //r[1] = wayPoints[wpNum].longitude - sensorData.longi;
  normr = sqrt(pow(r[0],2)+pow(r[1],2));

  //If the boat has reached its waypoint, starts calculating short term navigation towards next waypoint
  //But only if there is another waypoint to look at
  //if (normr < wpErr) {
  mwSize size = 100;
  if (normr < 1 && (wpNum + 2) < size) {
    wpNum = wpNum + 2;
    //wpNum = wpNum + 1;
    r[0] = wayPoints[wpNum] - sensorData[0];
    r[1] = wayPoints[wpNum+1] - sensorData[1];
    //r[0] = wayPoints[wpNum].latitude - sensorData.lati;
    //r[1] = wayPoints[wpNum].longitude - sensorData.longi;
    normr = sqrt(pow(r[0],2)+pow(r[1],2));
  }
  mwSize deg = (mwSize) floor(atan(r[1]/r[0])*(180.0/PI)) - windDir;
  if(deg < 0) { deg = deg + 360; }
  
  if (polarPlot[(deg)] < -10){
      deg = (mwSize) floor(deg + windDir);
      if(deg > 360){ deg = deg - 360; }
      nextHeading = deg;
  }
  else{  
      // calculate n
      n = 50*(1 + 5/normr);
      //n = 1 + Pc/normr;
      //  
      // step through right half of polar plot first
      alpha = 360;
      v_T_R_max = 0;
      thetaBoat_R_max = windDir + alpha;
      //thetaBoat_R_max = sensorData.windDir + alpha;

      while (alpha >= 180){
        // calculate vB
        vB[0] = polarPlot[(alpha)] * cos((alpha + windDir)*(PI/180.0));
        vB[1] = polarPlot[(alpha)] * sin((alpha + windDir)*(PI/180.0));
        //vB[0] = polarPlot[(alpha)] * cos((sensorData.windDir + alpha)*(PI/180.0));
        //vB[1] = polarPlot[(alpha)] * sin((sensorData.windDir + alpha)*(PI/180.0));

        // calculate velocity towards target
        v_T_R = vB[0] * (r[0]/normr) + vB[1] * (r[1]/normr);
        //v_T_R = vB[0] * (r[0]/normr) + vB[1] + (r[1]/normr); //this may not be correct and needs to be fixed.  needs to be matrix mult where vB is 1x2 and r is 2x1

        // check if this is largest velocity, if it is save it
        if (v_T_R > v_T_R_max){
          v_T_R_max = v_T_R;
          thetaBoat_R_max = windDir + alpha;
          //thetaBoat_R_max = sensorData.windDir + alpha;
        }
        alpha = alpha - delalpha;
      }

      //now run for left half of polar plot
      alpha  = 0;
      v_T_L_max = 0;
      thetaBoat_L_max = windDir + alpha * (PI/180.0);
      //thetaBoat_L_max = sensorData.windDir + alpha * (PI/180.0);

      while (alpha <= 180){
        //calculate vB 
        vB[0] = polarPlot[(alpha)] * cos((windDir + alpha)*(PI/180.0));
        vB[1] = polarPlot[(alpha)] * sin((windDir + alpha)*(PI/180.0));
        //vB[0] = polarPlot[(alpha)] * cos((sensorData.windDir + alpha)*(PI/180.0));
        //vB[1] = polarPlot[(alpha)] * sin((sensorData.windDir + alpha)*(PI/180.0));
        //    
        //calculate velocity towards target
        v_T_L = vB[0] * (r[0]/normr) + vB[1] * (r[1]/normr);
        //v_T_L = vB[0] * (r[0]/normr) + vB[1] + (r[1]/normr);
        //    
        //    //check if this is largest velocity, if it is save it
        if (v_T_L > v_T_L_max){
          v_T_L_max  = v_T_L;
          thetaBoat_L_max  = windDir + alpha;
          //thetaBoat_L_max  = sensorData.windDir + alpha;
        }
        alpha  = alpha + delalpha;
      }

      //  
      //  //choose either right or left heading
      d1 = fabsf(thetaBoat_R_max - boatDir);
      if(d1 > 180){
          d1 = 360 - d1;
      }

      d2 = fabsf(thetaBoat_L_max - boatDir);
      if(d2 > 180){
          d2 = 360 - d2;
      }

      //if (abs(thetaBoat_R_max - sensorData.boatDir) < abs(thetaBoat_L_max - sensorData.boatDir)){
      if (d1 < d2){
        if ((v_T_R_max * n) < v_T_L_max){
          nextHeading = thetaBoat_L_max;
          opt = 1;
        } else {
          nextHeading = thetaBoat_R_max;
        }
      } else {
        if ((v_T_L_max * n) < v_T_R_max) {
          nextHeading = thetaBoat_R_max;
          opt = 2;
        } else {
          nextHeading = thetaBoat_L_max;
        }
      }
  }
  
  if (nextHeading > 360){
    nextHeading = nextHeading - 360; 
  }
  
//   if (opt == 1){
//       error = nextHeading - boatDir;
//   }
//   else if (opt == 2){
//       error = nextHeading - boatDir;
//   }
//   else{
//       error = nextHeading - boatDir;
//   }
  error = nextHeading - boatDir;
  //error = nextHeading - sensorData.boatDir;
  
  //make sure error is between 180 & -180
  if (error > 180 || error < -180){
    if (error > 180){
      error = error - 360;
    } else if (error < -180){
      error = error + 360;
    }
  }
  
  //calculate command from error using PD algorithm
  command  = error*1 + 0.5*(error-prevErr);
  //command  = error*controlP + controlD*(error-prevErr);
  prevErr = error;
  
  //set rudder angle based on command
  if (command > 0 || command < 0) {
    rudderAngle = -command;
  } else{
    rudderAngle  = 0;
  }
  
  float rudderMax = 15;
  //check that rudderAngle is not greater than rudderMax, if it is set it to rudderMax
  if (rudderAngle > rudderMax){
    rudderAngle = rudderMax;
  } else if (rudderAngle < -rudderMax){
    rudderAngle = -rudderMax;
  }
  
  if((opt == 1 || opt == 2) && fabsf(nextHeading - boatDir) >= 15
  && fabsf(nextHeading - boatDir - 360) >= 15){
    if(opt == 1){
        rudderAngle = 15;
    } else{
        rudderAngle = -15;
    }
    //prevErr = -180000;
  }
  
  sailAngle = windDir;
  //sailAngle=sensorData.windDir;
  
  //Serial.print("Waypoint #");
  //Serial.print(wpNum);
  //Serial.print(": ");
  //Serial.print(wayPoints[wpNum].latitude);
  //Serial.print(", ");
  //Serial.println(wayPoints[wpNum].longitude);
  //Serial.print("Next Heading: ");
  //Serial.println(nextHeading);
  //Serial.println();
  
  angles[0] = rudderAngle;
  angles[1] = sailAngle;
  //angles[1]= polarPlot[90];
  angles[2] = prevErr;
  //angles[3] = deg;
  angles[3] = wpNum;
}

// The gateway function
// This functions assists in creating a mex file. You run it by
// calling 'mex nShort.c' in your command window, which will create the
// mex file and thus allow you to call 'nShort(wayPoints,sensorData,windDir,boatDir,wpNum,prevErr)
// within any other MATLAB files in the same directory as the mex file
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{
    int wpNum;
    double *wPs;
    double *sD;
    float wDir;
    float bDir;
    float pError;
    float *angles;
    
    //double multiplier;              /* input scalar */
    //double *inMatrix;               /* 1xN input matrix */
    //size_t ncols;                   /* size of matrix */
    //double *outMatrix;              /* output matrix */

    /* check for proper number of arguments */
    if(nrhs!=6) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nrhs","Six inputs required.");
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
    wpNum = (int) mxGetScalar(prhs[5]);

    /* create a pointer to the real data in the input matrices  */
    wPs = mxGetPr(prhs[0]);
    sD = mxGetPr(prhs[1]);
    //sD = (float*) mxGetPr(prhs[1]);

    /* create the output matrix */
    plhs[0] = mxCreateNumericMatrix(1, 4, mxSINGLE_CLASS, mxREAL);
    //plhs[0] = mxCreateDoubleMatrix(1,(mwSize)ncols,mxREAL);

    /* get a pointer to the real data in the output matrix */
    angles = (float *) mxGetData(plhs[0]);

    /* call the computational routine */
    nShort(wPs,sD,wDir,bDir,pError,(mwSize)wpNum,angles);
}