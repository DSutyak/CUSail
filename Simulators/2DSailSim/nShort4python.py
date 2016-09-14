
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
void nShort(double *wayPoints, double *sensorData, float windDir, float boatDir, mwSize wpNum, float jibing, float tackpointx, float tackpointy, float *angles) {
  
    
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
  //*****

  printf("windidr: %f\n",windDir);

  r[0] = wayPoints[wpNum] - sensorData[0];
  r[1] = wayPoints[wpNum+1] - sensorData[1];
  w[0] = cos((windDir)*(PI/180.0));
  w[1] = sin((windDir)*(PI/180.0));
  //r[0] = wayPoints[wpNum].latitude - sensorData.lati;
  //r[1] = wayPoints[wpNum].longitude - sensorData.longi;
  normr = sqrt(pow(r[0],2)+pow(r[1],2));


  //latitude longitude distance


  //If the boat has reached its waypoint, starts calculating short term navigation towards next waypoint
  //But only if there is another waypoint to look at
  //if (normr < wpErr) {
  float size = 10.0;
  // printf("distance to waypoint %f; size of array %f; wpnum %f \n",normr,size,wpNum );
  if (normr < 2 && ((wpNum + 2) < size)) {
    printf("reached waypoint\n");
    wpNum = wpNum + 2;
    //wpNum = wpNum + 1;
    r[0] = wayPoints[wpNum] - sensorData[0];
    r[1] = wayPoints[wpNum+1] - sensorData[1];
    //r[0] = wayPoints[wpNum].latitude - sensorData.lati;
    //r[1] = wayPoints[wpNum].longitude - sensorData.longi;
    normr = sqrt(pow(r[0],2)+pow(r[1],2));
  }
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

  printf("anglewaypoint: %f anglewaypoint",anglewaypoint);
  float dirangle=anglewaypoint-windDir;
  //converts to 0-360
  dirangle=(float)((int)dirangle%360);
  dirangle=dirangle+360;
  dirangle=(float)((int)dirangle%360);
  // printf("dirangle: %f\n",dirangle);


 
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


  printf("next heading:%f\n",nextHeading);

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
  //jibing 6: jibe


  float distance=pow(sensorData[0]-tackpointx,2)+pow(sensorData[1]-tackpointy,2);

  if(dirangle>180){
    dirangle-=360;
  }
  float jibedistance=70;
  float jibeangle=10;
  printf("tacking point: %f, %f\n",tackpointx,tackpointy);
  printf("dirangle: %f\n", dirangle);

  if (jibing == 0 || jibing==1){
    if (fabsf(dirangle)<optpolartop){
      //head to the left of the wind
      if(boatwind<0){
        jibing=2;
      }
      //head to the right of the wind
      else{
        jibing=3;
      }
    }
    else if(fabsf(dirangle)>180-optpolarbot){
      if(boatwind<0){
        jibing=4;
      }
      //head to the right of the wind
      else{
        jibing=5;
      }
    }
  }

  //if we are going upwind or we need to keep going upwind
  if((fabsf(dirangle)<optpolartop || fabsf(dirangle)>180-optpolarbot) && 
    (jibing==0 || jibing ==1 || jibing ==2 || jibing == 3 || jibing==4|| jibing==5)){
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
      // if the boat direction is not on the optimmum angle
      // correct the boat direction by turning the tail
      // if(fabsf(fabsf(boatwind)-optpolartop)>turnradius){
      //   //right of the wind
      //   if(boatwind>0){ 
      //     if(differenceheading>0){
      //       tailangle=windboat+tailangleofattack;
      //     }
      //     //if the heading is to the right of the boat
      //     else{
      //       tailangle=windboat-tailangleofattack;
      //     }
      //   }
      //   else{
      //     if(differenceheading>0){
      //       tailangle=windboat+tailangleofattack;
      //     }
      //     //if the heading is to the right of the boat
      //     else{
      //       tailangle=windboat-tailangleofattack;
      //     }
      //   }
      // }
      // else{
      //   tailangle=windboat;
      // }
      if(jibing==2){
        printf("up left of wind\n");
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
        printf("up right of wind\n");
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
        printf("down left of wind\n");
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
        printf("down right of wind\n");
        printf("check: %f",fabsf(fabsf(boatwind)-(180-optpolarbot)));
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

    printf("staying on heading past tacking point\n");
    if(boatwind>0){
        sailAngle=windboat+angleofattack;
      }
    else{
      sailAngle=windboat-angleofattack;
    }
    if(jibing==2){
      printf("up left of wind\n");
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
      printf("up right of wind\n");
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
    // else if (jibing==4){
    //   printf("down left of wind\n");
    //   if(fabsf(fabsf(boatwind)-(180-optpolarbot))<5){
    //       tailangle=windboat;
    //   }
    //   //if the boat is to the right of optpolartop
    //   else if(boatwind-(180-optpolarbot)<0){
    //     tailangle=windboat-tailangleofattack;
    //   }
    //   else{
    //     tailangle=windboat+tailangleofattack;
    //   }
    // }
    // else if (jibing==5){
    //   printf("down right of wind\n");
    //   if(fabsf(fabsf(boatwind)+(180-optpolarbot))<5){
    //       tailangle=windboat;
    //   }
    //   //if the boat is to the right of optpolartop
    //   else if(boatwind+(180-optpolarbot)<0){
    //     tailangle=windboat+tailangleofattack;
    //   }
    //   else{
    //     tailangle=windboat-tailangleofattack;
    //   }
    // }
  }
  //if we reach the point that we can start jibing
  else if((distance >= jibedistance && (jibing == 2 || jibing == 3)) || (fabsf(dirangle)>180-optpolarbot && ( jibing == 4 || jibing==5))){
  //else if(jibing==4){
    printf("start jibing: %f, x: %f, y: %f\n",distance,tackpointx,tackpointy);
    sailAngle=windboat;
    if (differenceheading>0){
      tailangle=windboat+tailangleofattack;
    }
    else{
      tailangle=windboat-tailangleofattack;
    }
    jibing = 6;
  }
  else if(jibing ==6){
    sailAngle=windboat;
    if(differenceheading>0){
      tailangle=windboat+tailangleofattack*2;
    }
    //if the heading is to the right of the boat
    else{
      tailangle=windboat-tailangleofattack*2;
    }
    //succesfully completed jibe
    if(fabsf(differenceheading)<30){
      jibing=1;
      tackpointx=0;
      tackpointy=0;

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
  else{
    printf("general turn code\n");
  //else if(jibing==1){
      //if the heading is to the right of the wind
      if(headingwind>0){
        printf("heading to the right\n");
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
            // if (fabs(boatwind)<90){

            //   sailAngle=windboat-angleofattack/5;
            // }
            // else{
            //   sailAngle=windboat;
            // }
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
            // if (fabs(boatwind)<90){

            //   sailAngle=windboat+angleofattack/5;
            // }
            // else{
            //   sailAngle=windboat;
            // }
            sailAngle=windboat;
            tailangle=windboat-tailangleofattack*2;
          }
    }


  }
  tailangle=tailangle*-1;
  sailAngle=sailAngle*-1;

  
  angles[0] = tailangle;
  angles[1] = sailAngle;
  angles[2] = jibing;
  angles[3] = jibing;
  angles[4] = tackpointx;
  angles[5] = tackpointy;


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
    int wpNum;
    double *wPs;
    double *sD;
    float wDir;
    float bDir;
    float pError;
    float tackpointx;
    float tackpointy;
    float *angles;
    
    //double multiplier;              /* input scalar */
    //double *inMatrix;               /* 1xN input matrix */
    //size_t ncols;                   /* size of matrix */
    //double *outMatrix;              /* output matrix */

    /* check for proper number of arguments */
    if(nrhs!=8) {
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
    tackpointx = (float) mxGetScalar(prhs[6]);
    tackpointy = (float) mxGetScalar(prhs[7]);

    /* create a pointer to the real data in the input matrices  */
    wPs = mxGetPr(prhs[0]);
    sD = mxGetPr(prhs[1]);
    //sD = (float*) mxGetPr(prhs[1]);

    /* create the output matrix */
    plhs[0] = mxCreateNumericMatrix(1, 6, mxSINGLE_CLASS, mxREAL);
    //plhs[0] = mxCreateDoubleMatrix(1,(mwSize)ncols,mxREAL);

    /* get a pointer to the real data in the output matrix */
    angles = (float *) mxGetData(plhs[0]);

    /* call the computational routine */
    nShort(wPs,sD,wDir,bDir,pError,(mwSize)wpNum, tackpointx, tackpointy, angles);
}
