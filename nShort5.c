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
void nShort(double *wayPoints, double *sensorData, float windDir, float boatDir, float wpNum, float waypointsize, float jibing, float tackpointx, float tackpointy, float *angles) 
  
  printf("\n\n");
  
  //Serial.println("Short Term Navigation");
  float nextHeading;
  float error;
  float command;
  mwSize alpha;
  float n;
  float normr; //normal distance to the waypoint
  float v_T_R_max;
  float v_T_L_max;
  float v_T_R;
  float v_T_L;
  float thetaBoat_R_max;
  float thetaBoat_L_max;
  float vB[2];
  float r[2]; //r[0]: Longitude difference b/w current position and waypoint, 
              //r[1]: Latitude difference b/w current position and waypoint 
  float w[2]; //w[0]: Cosine of the wind direction w.r.t North,
              //w[1]: Sine of the wind direction w.r.t North
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
    printf("new wpnum: %f\n", waypointnumber);

    //reset variables because we are close to a waypoint
    r[0] = wayPoints[waypointnumber] - sensorData[0];
    r[1] = wayPoints[waypointnumber+1] - sensorData[1];
    w[0] = cos((windDir)*(PI/180.0));
    w[1] = sin((windDir)*(PI/180.0));
    oldnormr=normr;
    normr = sqrt(pow(r[0],2)+pow(r[1],2));
    printf("new normr: %f\n",normr );
  }


  //latitude longitude distance


  // printf("waypoint: %f, %f\n",wayPoints[wpNum],wayPoints[wpNum+1]);
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
  // printf("angle north to waypoint: %f\n",anglewaypoint);

  //andgle from the wind to the waypoint vector. this is used to determine if we can sail to the next heading
  //or if we have to tack/jibe

  // printf("anglewaypoint: %f anglewaypoint",anglewaypoint);
  float dirangle=anglewaypoint-windDir;
  //converts to 0-360
  dirangle=(float)((int)dirangle%360);
  dirangle=dirangle+360;
  dirangle=(float)((int)dirangle%360);
  // printf("dirangle: %f\n",dirangle);


  // float 2r[2];
  // if((waypointnumber + 2) < size){
  //   2r[0] = wayPoints[waypointnumber+2] - sensorData[0];
  //   2r[1] = wayPoints[waypointnumber+3] - sensorData[1];
  // }
  // else{
  //   2r[0] = wayPoints[waypointnumber] - sensorData[0];
  //   2r[1] = wayPoints[waypointnumber+1] - sensorData[1];
  // }
  // float 2anglewaypoint=atan2(r[0],r[1])*360/(2*PI);
  // 2anglewaypoint=(float)((int)anglewaypoint%360);
  // 2anglewaypoint=anglewaypoint+360;
  // 2anglewaypoint=(float)((int)anglewaypoint%360);
  // float 2dirangle=2anglewaypoint-windDir;
 
  boatDir=360-(boatDir-90);
  boatDir=(float)((int)boatDir%360);
  boatDir=boatDir+360;
  boatDir=(float)((int)boatDir%360);
   printf("boat orientation: %f\n",boatDir);
    //right=0, left =1
     int side;



  //we have to port tack into the wind
  //right up
  if(dirangle<optpolartop){
    // printf("right up port tack");
    nextHeading=optpolartop;
  }
  //we can sail directly to the target
  //right side
  else if(dirangle<(180-optpolarbot)){
    // printf("right side");
    nextHeading=dirangle;
  }
  //we have to port jibe
  //right down
  else if(dirangle<(180)){
    // printf("right down port jibe");
    nextHeading=180-optpolarbot;
  }
  //we have to starboard jibe
  //left down
  else if(dirangle<(180+optpolarbot)){
    // printf("left down starboard jibe");
    nextHeading=180+optpolarbot;
  }
  //can sail directly to the target
  //left side
  else if(dirangle<(360-optpolartop)){
    // printf("left side");
    nextHeading=dirangle;
  }
  //starboard tack
  //left up
  else{
    // printf("left up starboard tack");
    nextHeading=360-optpolartop;
  }

  // printf("next heading wrt wind:%f\n",nextHeading);
  //convert back to wrt North
  nextHeading=nextHeading+windDir;
  //converts to 0-360
  nextHeading=(float)((int)nextHeading%360);
  nextHeading=nextHeading+360;
  nextHeading=(float)((int)nextHeading%360);

  //convert to -180 to 180
  if(nextHeading>180){
    nextHeading=nextHeading-360;
  }
  if(windDir>180){
    windDir=windDir-360;
  }
  if(boatDir>180){
    boatDir=boatDir-360;
  }

  //convert to counter clockwise direction
  // nextHeading=-1*nextHeading;
  // windDir=-1*windDir;
  // boatDir=-1*boatDir;


  // printf("next heading:%f\n",nextHeading);

  int headingT[23]={57,62,67,72,77,82,87,92,97,102,107,112,117,122,126,131,135,139,143,148,153,158,163};
  int sailT[23]=   {35,40,45,50,55,60,65,70,75,80,85,90,95,100,105,110,115,120,125,131,137,144,150};
  int tailT[23]=   {10,11,11,12,12,12,13,13,13,13,14,14,15,15,15,15,15,15,15,15,15,15,15};

  float turnradius=5;
  //greater means turn left
  float differenceheading=boatDir-nextHeading;
  float boatwind=boatDir-windDir;
  float tailangleofattack=15;

  float headingwind=nextHeading-windDir;
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

  float angleofattack=15;


  //convert to counter clockwise direction


  //sail angle directly into the wind
  float windboat=windDir-boatDir;


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

  printf("waypoints at: %f, %f\n", wayPoints[waypointnumber],wayPoints[waypointnumber+1]);

  float distance=pow(sensorData[0]-tackpointx,2)+pow(sensorData[1]-tackpointy,2);

  if(dirangle>180){
    dirangle-=360;
  }
  //change jibedistance to 10 for arduino
  float jibedistance=70;
  float jibeangle=10;
  // printf("tacking point: %f, %f\n",tackpointx,tackpointy);
  // printf("dirangle: %f\n", dirangle);


  //starting code,reached intermediate waypoint, reached last waypoint
  if (jibing == 0 || jibing==1 || oldnormr<detectionradius){
    //close to last waypoint
    if((waypointnumber + 2) >= size && (oldnormr<detectionradius || normr<detectionradius)){
      tackpointx=sensorData[0];
      tackpointy=sensorData[1];
      printf("close to last waypoint, setting jibing\n");
      if(boatwind>0){
        jibing=3;
      }
      else{
        jibing=2;
      }
    }
    //reset jibing to new heading
    else{
      if (fabsf(dirangle)<optpolartop){

        //head to the left of the wind
        if(boatwind<0){
          printf("set tack to 2\n");
          jibing=2;
        }
        //head to the right of the wind
        else{
          printf("set tack to 3\n");
          jibing=3;
        }
      }
      else if(fabsf(dirangle)>180-optpolarbot){
        if(boatwind<0){
          printf("set tack to 4\n");
          jibing=4;
        }
        //head to the right of the wind
        else{
          printf("set tack to 5\n");
          jibing=5;
        }
      }
      //heading to right, boat to left of wind
      else if(headingwind>0){
        //boat to left of wind
        if(boatwind<0){
          printf("set tack to 6\n");
          jibing=6;
        }
      }
      //heding to left, boat to right of wind
      else if(headingwind<0){
        if(boatwind>0){
          printf("set tack to 7\n");
          jibing=7;
        }
      }
    }
  }


  //check if we are way off our waypoint (possible extra input parameter) and if so then reset jibing
  //go through two points in a direction to simulate around buoy
  //bump in heading upwind (around tack point?)
  //turning wrong when for a sec when jibing
  //optimize variables
  //heading at start depends on location of next next heading (eg if we want to go around it to the right or to the left)
  //start tacking before getting to the tacking point
  //immediate sail upwind for station keeping

  //opposite direction of second buoy
  //have to clear both lines
  //have we cleared the first line? if so, then keep going to it, and then check once we've actually hit it


  //DONE
  //plotting points

  //for jibing, its possible it gets turned around too much at the start, which is why it does the full turn. if it gets on the same side
  //but 30 degrees past the heading, stop

  // if(jibing==3 && (windDir>45|| windDir<anglewaypoint) ||
  //   jibing==2 && (windDir<-45|| windDir>anglewaypoint) )


  //if we are going upwind or we need to keep going upwind
  //possibly redundant first boolean
  printf("dirangle: %f",dirangle);
  if((fabsf(dirangle)<optpolartop || fabsf(dirangle)>180-optpolarbot) &&
    (jibing ==2 || jibing == 3 || jibing==4|| jibing==5)){
      printf("go at a tack: %f\n",jibing);
      // jibing=2;
      tackpointx=sensorData[0];
      tackpointy=sensorData[1];
      if(boatwind>0){
        sailAngle=windboat+angleofattack;
      }
      else{
        sailAngle=windboat-angleofattack;
      }
      if(jibing==2){
        // printf("up left of wind\n");
        if(fabsf(fabsf(boatwind)-optpolartop)>30){
          sailAngle=windboat;
        }
        //on correct heading
        if(fabsf(fabsf(boatwind)-optpolartop)<5){
            tailangle=windboat;
        }
        //if the boat is to the right of optpolartop
        else if(boatwind+optpolartop<0){
          tailangle=windboat-tailangleofattack;
        }
        else{
          tailangle=windboat+tailangleofattack;
        }

      }
      //jibing==3
      else if (jibing==3){
        // printf("up right of wind\n");
        if(fabsf(fabsf(boatwind)-optpolartop)>30){
          sailAngle=windboat;
        }
        if(fabsf(fabsf(boatwind)-optpolartop)<5){
            tailangle=windboat;
        }
        //if the boat is to the right of optpolartop
        else if(boatwind-optpolartop<0){
          tailangle=windboat-tailangleofattack;
        }
        else{
          tailangle=windboat+tailangleofattack;
        }
      }
      else if (jibing==4){
        // printf("down left of wind\n");
        if(fabsf(fabsf(boatwind)-(180-optpolarbot))>30){
          sailAngle=windboat;
        }
        if(fabsf(fabsf(boatwind)-(180-optpolarbot))<5){
            tailangle=windboat;
        }
        //if the boat is to the left of optpolarbot - (-180+40)
        else if(boatwind+(180-optpolarbot)<0){
          tailangle=windboat-tailangleofattack;
        }
        else{
          tailangle=windboat+tailangleofattack;
        }
      }
      else if (jibing==5){
        // printf("down right of wind\n");
        if(fabsf(fabsf(boatwind)-(180-optpolarbot))>30){
          sailAngle=windboat;
        }
        if(fabsf(fabsf(boatwind)-(180-optpolarbot))<5){
            tailangle=windboat;
        }
        //if the boat is to the right of optpolarbot
        else if(boatwind-(180-optpolarbot)<0){
          tailangle=windboat+tailangleofattack;
        }
        else{
          tailangle=windboat-tailangleofattack;
        }
      }
  }
  //stay upwind and head past the tacking point
  else if(fabsf(dirangle)>optpolartop && (jibing==2 || jibing == 3) && distance<jibedistance){
  //else if(jibing ==3){

    // jibing=4;

    //might want to check if outside 30 degrees

    printf("staying on heading past tacking point: %f\n",jibing);
    if(boatwind>0){
        sailAngle=windboat+angleofattack;
      }
    else{
      sailAngle=windboat-angleofattack;
    }
    if(jibing==2){
      // printf("up left of wind\n");
      //on correct heading
      if(fabsf(fabsf(boatwind)-optpolartop)<5){
          tailangle=windboat;
      }
      //if the boat is to the right of optpolartop
      else if(boatwind+optpolartop<0){
        tailangle=windboat-tailangleofattack;
      }
      else{
        tailangle=windboat+tailangleofattack;
      }

    }
    //jibing==3
    else if (jibing==3){
      // printf("up right of wind\n");
      if(fabsf(fabsf(boatwind)-optpolartop)<5){
          tailangle=windboat;
      }
      //if the boat is to the right of optpolartop
      else if(boatwind-optpolartop<0){
        tailangle=windboat-tailangleofattack;
      }
      else{
        tailangle=windboat+tailangleofattack;
      }
    }
  }
  //if we reach the point that we can start jibing
  else if((distance >= jibedistance && (jibing == 2 || jibing == 3)) || (fabsf(dirangle)<180-optpolarbot && ( jibing == 4 || jibing==5))){
  //else if(jibing==4){
    printf("start jibing: %f, x: %f, y: %f\n",distance,tackpointx,tackpointy);
    sailAngle=windboat;
    if (differenceheading>0){
      tailangle=windboat+tailangleofattack;
    }
    else{
      tailangle=windboat-tailangleofattack;
    }
    if(jibing==2 || jibing==4){
      jibing=6;
    }
    else if (jibing==3 || jibing==5){
      jibing=7;
    }
  }

  //we are on our heading
  else if (fabsf(differenceheading)<  turnradius){
    printf("sail directly to waypoint\n");
  //else if(jibing==0){
    jibing=0;
    //set tail into wind
    tailangle=windboat;
    //generate lift, if wind is from right side vs left side angle of the attack is + or -
    if(boatwind>0){
      sailAngle=windboat+angleofattack;
    }
    else {
      sailAngle=windboat-angleofattack;
    }

  }
  else if(jibing==6){
    if(fabsf(differenceheading)<30){
      jibing=0;
    }
              
    printf("jibe left\n");
    sailAngle=windboat;
    tailangle=windboat+tailangleofattack*2;
  }
  else if(jibing==7){
    if(fabsf(differenceheading)<30){
      jibing=0;
    }
    printf("jibe right\n");
    sailAngle=windboat;
    tailangle=windboat-tailangleofattack*2;
  }
  else{
    printf("general turn code\n");
  //else if(jibing==1){
      //if the heading is to the right of the wind
      if(headingwind>0){
        // printf("heading to the right\n");
          //if the boat is to the right of the wind
          if(boatwind>0){
              //forward lift
              //if we are way off course, set sail into wind so we don't drift towards wrong heading
              if(fabsf(differenceheading)>30){
                sailAngle=windboat;
                if(differenceheading>0){
                  tailangle=windboat+tailangleofattack*2;
                }
                //if the heading is to the right of the boat
                else{
                  tailangle=windboat-tailangleofattack*2;
                }
              }
              else{
                //succesfully excecuted jibe
                jibing=1;
                tackpointx=0;
                tackpointy=0;
                sailAngle=windboat+angleofattack;
              
                //if the heading is to the left of the boat
                if(differenceheading>0){
                  tailangle=windboat+tailangleofattack;
                }
                //if the heading is to the right of the boat
                else{
                  tailangle=windboat-tailangleofattack;
                }
            }
          }
          //if the boat is to the left of the wind
          //need to turn drastically
          else{
            sailAngle=windboat;
            tailangle=windboat+tailangleofattack*2;
          }

      }
      //if the heading is to the left of the wind
      else{
          //if the boat is to the left of the wind
          if(boatwind<0){
              //forward lift
              //if we are way off course, set sail into wind so we don't drift towards wrong heading
              if(fabsf(differenceheading)>30){
                // printf("checking\n");
                sailAngle=windboat;
                if(differenceheading>0){
                  tailangle=windboat+tailangleofattack*2;
                }
                //if the heading is to the right of the boat
                else{
                  tailangle=windboat-tailangleofattack*2;
                }
              }
              else{

                //succesfully excecuted jibe
                jibing=1;
                tackpointx=0;
                tackpointy=0;
                sailAngle=windboat-angleofattack;
                //if the heading is to the left of the boat
                if(differenceheading>0){
                  tailangle=windboat+tailangleofattack;
                }
                //if the heading is to the right of the boat
                else{
                  tailangle=windboat-tailangleofattack;
                }
              }
          }
          //if the boat is to the right of the wind
          //need to turn drastically
          else{
            sailAngle=windboat;
            tailangle=windboat-tailangleofattack*2;
          }
    }


  }

  printf("tailangle: %f\n",tailangle);
  printf("sailAngle: %f\n", sailAngle);
  tailangle=tailangle*-1;
  sailAngle=sailAngle*-1;

  
  angles[0] = tailangle;
  angles[1] = sailAngle;
  angles[2] = jibing;
  angles[3] = jibing;
  angles[4] = tackpointx;
  angles[5] = tackpointy;
  angles[6] = waypointnumber;


//code to look up jesses table
    // if(false){
    // for (int i=0;i<23;i++){
    //   if(fabsf(headingT[i]-nextHeading)<2.5){
    //     printf("sail to heading: %d\n", headingT[i]);
    //     sailAngle=sailT[i];
    //     tailangle=tailT[i];
    //   }
    //   if(fabsf(-1*headingT[i]-nextHeading)<2.5){
    //     printf("sail to heading: %d\n", headingT[i]);
    //     sailAngle=-1*sailT[i];
    //     tailangle=-1*tailT[i];
    //   }
    // }
    // //not in range
    // if(nextHeading < 57 && nextHeading>0){
    //   printf("sail to right up heading: \n");
    //   sailAngle=35;
    //   tailangle=10;
    // } else if (nextHeading < 0 && nextHeading > -57){
    //   printf("sail to left up heading: \n");
    //   sailAngle=-35;
    //   tailangle=-10;
    // } else if (nextHeading > 163){
    //   printf("sail to right down heading: \n");
    //   sailAngle=150;
    //   tailangle= 15;
    // } else if (nextHeading < -163){
    //   printf("sail to left down heading: \n");
    //   sailAngle= -150;
    //   tailangle= -15;
    // }
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
