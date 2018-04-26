#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include "print.h"
#include "sensors.h"
#include "navigation.h"
#include "navigation_helper.h"

//Define Servo types for Sail and Tail
Servo tailServo;
Servo sailServo;

/*----------Navigation Variables----------*/
unsigned long  milTime; //Miliseconds since program started
int wpNum; //the current waypoint's number in the wayPoints array
int numWP; //total number of waypoints on current course
float detectionradius = 10; //how far away the boat marks a waypoint "reached"
float optpolartop = 60; //Optimal upwind angle for waypoints requiring a tack
float optpolarbot = 60; //Optimal downwind angle for waypoints requiring a tack
float angleofattack = 10;
coord_xy wayPoints[maxPossibleWaypoints]; //the array containing the waypoints
float normr; //normal distance to the waypoint
float r[2]; //r[0]: Longitude difference b/w current position and waypoint,
            //r[1]: Lattitude difference b/w current position and waypoint
float w[2]; //w[0]: Cosine of the wind direction w.r.t North,
            //w[1]: Sine of the wind direction w.r.t North
float sailAngle;
float tailAngle;

// the angle that we intend to sail at WRT North
float intended_angle;
float intended_angle_of_attack;
// latitude is y, longitude is x
float max_distance=100;

//set maximum allowable width for boat to sail within
float upperWidth = 5;
float lowerWidth = 5;
bool isTacking = false;

//Stores information about current heading and tack. quadrant expects 0 for up, 1 for direct or turn, and 2 for bottom. rightLeft expects true for left and false for right
int quadrant;
bool rightLeft;

//set to 0 if we are not doing station keeping or 1 if we are
bool stationKeeping = 0;

// need to read these values from the arduino
float time_inside=0;
// time required to stay inside the box for station keeping
// 5 minutes*60 seconds*1000millis=300,000
// 4:25*60 seconds*1000millis=
float time_req=285000;

// 3 minutes*60 seconds*1000millis=300,000
float max_turn_time=180000;
float start_turn_time=0;
float start_box_time=0;
// 10 seconds*1000 millis=10000
float time_to_turn=18000;
float prev_intended_angle=270;
float prev_intended_angle_of_attack=10;
bool turning=false;
bool turn_finished=true;
bool start_end_wp=false;
bool continue_end_wp=false;
float final_intended_angle;

//set to true when doing , when we reach the last waypoint, reset wpNum to 0
bool endurance=false;

//set to true when doing obstacle avoidance
bool avoid_test=false;
bool avoiding=false;
float avoid_intended;
//the angle that we change our intended angle by in order to avoid an object
float turn_angle=1;
//1 is right, -1 is left, 0 is not seen yet
int avoid_direction=0;

bool search=false; //set to true when doing search and navigate.
// float left_bank=42

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
  milTime = millis();
}

/*Sets servos to the sailAngle and tailAngle*/
void nServos(void) {
  tailServo.write(tailAngle);
  sailServo.write(sailAngle);
}

void updateTime(void){
  milTime = millis();
}

void startTurn(float init_intended, float init_angle, unsigned long start_time){
  turning=true;
  prev_intended_angle=init_intended;
  prev_intended_angle_of_attack=init_angle;
  start_turn_time=start_time;
}

void holdTurn(){
  intended_angle=prev_intended_angle;
  intended_angle_of_attack=prev_intended_angle_of_attack;
}

void finishTurn(){
  turning=false;
  turn_finished=true;
}

void avoidObject(void) {
   addObjects();
   if (objectVals[1] != 400.0 && objectVals[0] != 400.0) {
    // if (!avoiding){
      avoiding=true;
      double initialReading = objectVals[1];
      double recentReading = objectVals[0];
      // double courseChange = initialReading - recentReading;
      double courseChange=recentReading-initialReading;
      //if its the first time we have seen an object
      if (avoid_direction==0){
        //right to left
        if (initialReading>150){
          avoid_direction=1;
        }
        //left to right
        else{
          avoid_direction=-1;
        }
      }
      avoid_intended+=avoid_direction*turn_angle;
      avoid_intended=(float)(((int)avoid_intended+360) %360);
      Serial1.println();
      Serial1.print("Intended: ");
      Serial1.print(avoid_intended);
  }
  //we cannot detect an object, so stop the temporary intended angle and sail
  // towards the target
  else {
    avoiding=false;
  }

  // tailAngle = (float)(
}

/*Method to determine whether the boat is above the greater tacking bound, for use in nShort to determine when to tack */
bool aboveBounds(float upperWidth, coord_xy point1, coord_xy point2){
    float angle = angleToTarget(point1, point2);
    float dy = fabs(upperWidth/tan(angle));
    float slope = xySlope(point1, point2);
    return (sensorData.x * slope + dy < sensorData.y);
}
/*Method to determine whether the boat is below the lesser tacking bound, for use in nShort to determine when to tack */
bool belowBounds(float lowerWidth, coord_xy point1, coord_xy point2){
    float angle = angleToTarget(point1, point2);
    float dy = fabs(lowerWidth/tan(angle));
    float slope = xySlope(point1, point2);
    return (sensorData.x * slope - dy > sensorData.y);
}

/*Method to determine sail and tail angle. The quadrant field expects values between 0 and 3, with 0=up, 1=direct or turn, and 2=bottom. rightLeft expects false for right and true for left*/
void nav(int quadrant, bool rightLeft, float windDir, float anglewaypoint){
   if(quadrant > 2){
      Serial1.print("Invalid argument sent to nav");
   }
   else if(quadrant==0){
      if(rightLeft){
        //Up Left
        Serial1.print("UP LEFT");
        intended_angle = windDir - optpolartop;
        intended_angle_of_attack = -15;
      }
      else{
        //Up Right
        Serial1.print("UP RIGHT");
        intended_angle= windDir + optpolartop;
        intended_angle_of_attack = 15;
      }
   }
   else if(quadrant==1){
      if(rightLeft){
        //Direct Left
        Serial1.print("DIRECT LEFT");
        intended_angle = anglewaypoint;
        intended_angle_of_attack = -15;
      }
      else{
        //Direct Right
        Serial1.print("DIRECT RIGHT");
        intended_angle = anglewaypoint;
        intended_angle_of_attack = 15;
      }
   }
   else{
      if(rightLeft){
        //Bottom Left
        Serial1.print("BOTTOM LEFT");
        intended_angle = windDir + 180 + optpolarbot;
        intended_angle_of_attack = -15;
      }
      else{
        //Bottom Right
        Serial1.print("BOTTOM RIGHT");
        intended_angle = windDir + 180 - optpolarbot;
        intended_angle_of_attack = 15;
      }
   }
}

/*----------Stored Coordinates----------*/
//Coordinates in and around the Engineering Quad, Cornell university
coord_t olin = {42.445457, -76.484322}; //Olin Hall
coord_t hollister = {42.444259, -76.484435}; //Hollister Hall
coord_t outsideDuffield = {42.444254, -76.482660}; //Outside West entrance to Duffield Hall, atop the stairs
coord_t outsideThurston = {42.444228, -76.483666}; //In front of Thurston Hall
coord_t engQuadX = {42.444612, -76.483492}; //Center of Engineering Quad
coord_t northBottomStairs = {42.444735, -76.483275}; //Bottom of stairs, to the left of North entrance to Duffield Hall
coord_t engQuadRight = {42.4444792, -76.483244}; //Middle of the right sector, looking North, of the engineering quad
coord_t sundial = {42.4448630, -76.4835293}; //Sundial on the engineering quad
coord_t bottomRightIntersection= {42.444242, -76.483247}; //path intersection bottom right of quad
coord_t thurstonWestEntrance={42.444245, -76.484068}; //west entrance to thurston on the path
coord_t ctb{42.442539, -76.485019};
coord_t stairsToCTown{42.444057, -76.484228}; //top of stairs between bard and hollister

//Coordinates in and around the Cornell Sailing Center
coord_t lakeOut = {42.469386,-76.504690}; //Out in the lake, to the left of the Cornell Sailing Center
coord_t lakeOut4 = {42.469847,-76.504522}; //Out in the lake, to the left of the Cornell Sailing Center
coord_t lakeOut_beach_far = {42.469065,-76.506674}; //Out in the lake, to the right of the Cornell Sailing Center
coord_t lakeOut_beach = {42.470894,-76.504712}; //Out in the lake, to the right of the Cornell Sailing Center but further North
coord_t lakeOut_beach2 = {42.470535, -76.50489};
coord_t lakeOut_beach3 = {42.471124, -76.505757};
coord_t shore_beach = {42.470862,-76.503950}; //Beach, to the right of the Cornell Sailing Center
coord_t acrossDock = {42.465702, -76.524625}; //Across the lake, when looking from the far edge of the dock to the right of the Cornell Sailing Center
coord_t acrossBeach = {42.467918, -76.525419}; //Across the lake, when looking from the beach to the left of the Cornell Sailing Center
coord_t low_dock={42.468951,-76.502941}; //Right at the lower dock
coord_t high_dock={42.469552,-76.503353}; //High dock launch point

coord_t dirt_mound={42.454386, -76.475693};
coord_t appel_door={42.453915, -76.475891};
coord_t appel_farRight={42.454345, -76.474471};
coord_t appel_topRight={42.454715, -76.474551};
coord_t appel_topLeft={42.454865, -76.475672};
coord_t appel_bottomRight={42.453828, -76.474454};

// COORDINATES AT US NAVAL ACADEMY
coord_t end_0 = {38.982585,-76.477650};
coord_t end_1 = {38.982893, -76.477089};
coord_t end_2 = {38.982273, -76.476141};
coord_t end_3 = {38.982130, -76.475690};
coord_t end_4 = {38.982436, -76.475389};
coord_t end_5 = {38.982820, -76.475041};
coord_t end_6 = {38.983139, -76.474748};
coord_t end_7 = {38.983570, -76.474500};
coord_t end_8 = {38.984001, -76.475111};
coord_t end_9 = {38.984429, -76.475765};
coord_t end_10 = {38.984887, -76.476336};
coord_t end_11 = {38.985242, -76.476844};
coord_t end_12 = {38.985730, -76.477460};
coord_t end_13 = {38.985274, -76.477820};
coord_t end_14 = {38.984842, -76.478194};
coord_t end_15 = {38.984389, -76.478585};
coord_t end_16 = {38.984030, -76.479000};
coord_t end_17 = {38.983646, -76.478501};
coord_t end_18 = {38.983304, -76.478074};
coord_t end_19 = {38.982930, -76.477625};


/*Sets waypoints for navigation
* by creating the wayPoints array*/
void setWaypoints(void) {


  //Make the waypoint array
  numWP = 3;
  wpNum = 0;

  /**
   * The origin is the point at which all xy coordinates are centered.
   * all points must be inserted using xyPoint(yourWaypoint) to convert to xy coordinates
   */
  setOrigin(outsideThurston);
  wayPoints[0] = xyPoint(hollister);
  wayPoints[1] = xyPoint(outsideThurston);
  wayPoints[2] = xyPoint(sundial);



  /*
  // nav test with wind from 340

//Points for lake test on Cayuga
//  numWP = 9;
  numWP=8;
  wpNum = 0;

  wayPoints[0]= {42.470432, -76.505028}; //nav_0
  wayPoints[1]={42.470318, -76.504906}; //nav_1
  wayPoints[2]={42.470348, -76.504654}; //nav_2
  wayPoints[3]={42.470493, -76.504143}; //nav_3
  wayPoints[4]={42.47065, -76.504028}; //nav_4
  wayPoints[5]={42.470764, -76.504135}; //nav_5
  wayPoints[6]={42.470943, -76.504326};//nav_6
  wayPoints[7]={42.47094, -76.504745};  //nav_7
  //wayPoints[8]={42.471104, -76.504883}; //nav_8

  */



  //STATION KEEPING TEST on Cayuga
  /*
  numWP=3;
  wpNum=0;
  wayPoints[0]={42.470737, -76.504707}; //nav_0
  wayPoints[1]={42.470737, -76.504707}; //nav_1
  wayPoints[2]={42.470863, -76.503975}; //nav_2

  */
  /*
   //station keeping lakehouse test (on land)
  //going to lakehouse then back to picnic

  numWP=3;
  wpNum=0;
  wayPoints[0]={42.47049, -76.503212}; //nav_0
  wayPoints[1]={42.47049, -76.503212}; //nav_1
  wayPoints[2]={42.470783, -76.503357}; //nav_2
  */

  //endurance race on Cayuga
  /*

  numWP=16;
  wpNum=0;
  wayPoints[0]={42.474224, -76.507195};
  wayPoints[1]={42.473862, -76.508194};
  wayPoints[3]={42.473209, -76.510284};
  wayPoints[4]={42.472359, -76.51004};
  wayPoints[5]={42.471474, -76.509651};
  wayPoints[6]={42.470634, -76.509155};
  wayPoints[7]={42.469826, -76.508621};
  wayPoints[8]={42.469063, -76.508011};
  wayPoints[9]={42.469334, -76.507187};
  wayPoints[10]={42.469635, -76.506279};
  wayPoints[11]={42.469856, -76.505264};
  wayPoints[12]={42.470695, -76.505699};
  wayPoints[13]={42.471519, -76.506134};
  wayPoints[14]={42.472309, -76.506645};
  wayPoints[15]={42.473183, -76.50695};
  */
  /*
  Navigation challenge: set waypoints, assuming wind from above, in numerical order
  around the buoys represented by x's


                              8


                        x     7     x
                                                    6







        0    x                                  x      5

            1                                      4
                  2                           3

  */
}

/*----------------------------------------*/
/*----------NAVIGATION ALGORITHM----------*/
/*----------------------------------------*/

/*Uses sensorData.windDir, sensorData.boatDir to set sailAngle and tailAngle.
 * sailAngle and tailAngle are set in terms of servo command numbers, but are first
 * calculated in terms of angle w.r.t the boat direction*/
void nShort(void) {

  /*----------Unit testing setters-----------*/
    //sensorData.lati = 42.4441782290;
 // sensorData.lati=outsideThurston.latitude;
  // sensorData.longi=outsideThurston.longitude;
//    sensorData.longi = -76.4834140241;
  sensorData.windDir = 0;
   // sensorData.boatDir = 0;
    //sensorData.sailAngleNorth = 90;

  //find the normal distance to the waypoint

  r[0] = wayPoints[wpNum].x - sensorData.x;
  r[1] = wayPoints[wpNum].y - sensorData.y;
  w[0] = cos((sensorData.windDir)*(PI/180.0));
  w[1] = sin((sensorData.windDir)*(PI/180.0));
  coord_xy currentPosition = {sensorData.x, sensorData.y};

  normr = xyDist(wayPoints[wpNum], currentPosition);

  printData();
  printWaypointData();

  float anglewaypoint=angleToTarget(coord_xy({sensorData.x, sensorData.y}), wayPoints[wpNum]);


  anglewaypoint=convertto360(anglewaypoint);


  /*
  when not doing station keeping, set boolean stationKeeping to false

  station keeping code: if we have not reached the required time, keep going
  to the same waypoint over and over (we will never register hitting it)
  set the waypoint to be the middle of the square of the station
  once we reach the allotted time, increment our waypoint counter and
  head to the next waypoint which will be a waypoint which is outside the box
  */
  // Serial1.print("T:");
  // float milInt=milTime/1000;
  // Serial1.print(milInt);
  // Serial1.print("; ");
  if (stationKeeping){
    /* plan: we set our first waypoint to be the center of the box w detection radius 20m (40/2)
       if we are going to the first waypoint, set radius to 20.
       also, store the start time that we hit the waypoint, so we can accurately stay inside
       the box for 5 minutes
       once we hit, increase the waypoint number to 1, which will make our boat continue to
       sail to the center of the box
       once 5 minutes are up, increase the waypoint number to 2 and set station keeping to false
       in this way we will sail out the box to wp2 which is outisde the box
    */
    if (wpNum==0){
      detectionradius=25;
      if(normr < detectionradius){
        printHitWaypointData();
        // where we will determine where we aim to go at the end of the required amount of time
        wpNum += 1 ;

        //reset variables because we have reached the old waypoint

        r[0] = wayPoints[wpNum].x - sensorData.x;
        r[1] = wayPoints[wpNum].y - sensorData.y;
        w[0] = cos((sensorData.windDir)*(PI/180.0));
        w[1] = sin((sensorData.windDir)*(PI/180.0));
        currentPosition = {sensorData.x, sensorData.y};

        normr = xyDist(wayPoints[wpNum], currentPosition);
        start_box_time=milTime;
      }
    }
    //we have  reached the 5 minute time (current time - the time we entered the box at)
    else if(milTime- start_box_time >= time_req ){
      printHitWaypointData();
      // where we will determine where we aim to go at the end of the required amount of time
      wpNum += 1 ;

      //reset variables because we have reached the old waypoint
      r[0] = wayPoints[wpNum].x - sensorData.x;
      r[1] = wayPoints[wpNum].y - sensorData.y;
      w[0] = cos((sensorData.windDir)*(PI/180.0));
      w[1] = sin((sensorData.windDir)*(PI/180.0));
      currentPosition = {sensorData.x, sensorData.y};
      normr = xyDist(wayPoints[wpNum], currentPosition);
      stationKeeping=0;
    }

  }
    if(normr < detectionradius){
      if ((wpNum + 1) < numWP){
        Serial1.print("hit");
        wpNum += 1 ;

        //reset variables because we have reached the old waypoint
        r[0] = wayPoints[wpNum].x - sensorData.x;
        r[1] = wayPoints[wpNum].y - sensorData.y;
        w[0] = cos((sensorData.windDir)*(PI/180.0));
        w[1] = sin((sensorData.windDir)*(PI/180.0));
        currentPosition = {sensorData.x, sensorData.y};
        normr = xyDist(wayPoints[wpNum], currentPosition);
      }
      else{
        //if we are doing endurance, reset the wpNum to 0
        if (endurance){
          wpNum += 1 ;

          //reset variables because we have reached the old waypoint
          r[0] = wayPoints[wpNum].x - sensorData.x;
          r[1] = wayPoints[wpNum].y - sensorData.y;
          w[0] = cos((sensorData.windDir)*(PI/180.0));
          w[1] = sin((sensorData.windDir)*(PI/180.0));

          currentPosition = {sensorData.x, sensorData.y};
          normr = xyDist(wayPoints[wpNum], currentPosition);

          wpNum=0;
        }
      }
    }
  }

  anglewaypoint=convertto360(anglewaypoint);

  float dirangle=anglewaypoint-sensorData.windDir;
  dirangle=convertto360(dirangle);

  float boatDirection = sensorData.boatDir;
  boatDirection=convertto360(boatDirection);

  // Wind w.r.t the boat
  float windboat;
  windboat=sensorData.windDir-boatDirection;

  float boat_wrt_wind=boatDirection-sensorData.windDir;
//  boat_wrt_wind=360-(boat_wrt_wind-90);
  boat_wrt_wind=convertto360(boat_wrt_wind);

  float windDir=sensorData.windDir;

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

  //Boat hits upper bound, tack right
  if(wpNum != 0 && aboveBounds(upperWidth, wayPoints[wpNum-1], wayPoints[wpNum])){
    Serial1.print("HIT UPPER BOUND, TACK RIGHT");
    if(!isTacking){
      quadrant = quadrant;
      rightLeft = !rightLeft;
    }
    isTacking = true;
  }
  //Boat hits lower bound, tack left
  else if(wpNum != 0 && belowBounds(lowerWidth, wayPoints[wpNum-1], wayPoints[wpNum])){
    Serial1.print("HIT LOWER BOUND, TACK LEFT");
    if(!isTacking){
      quadrant = quadrant;
      rightLeft = !rightLeft;
    }
    isTacking = true;
  }
  //  boat initially facing right
  else if (boat_wrt_wind<180) {
    if (dirangle<optpolartop && dirangle>0){
      Serial1.print("RIGHT UP RIGHT->");
      quadrant = 0;
      rightLeft = false;
      isTacking = false;
    }
    //Head directly to target to the right
    else if (dirangle>optpolartop && dirangle<180- optpolarbot){
      Serial1.print("RIGHT DIRECT RIGHT->");
      quadrant = 1;
      rightLeft = false;
      isTacking = false;
    }
    //Head directly to target to the left
    else if (dirangle>optpolarbot + 180 && dirangle<360 -optpolartop){
      Serial1.print("RIGHT DIRECT LEFT->");
      // turning
      // THIS IS WHERE WE WILL NEED TO CALL A TURN FUNCTION
//      start_box_time+=3000;
//      delay(3000);
      quadrant = 1;
      rightLeft = true;
      isTacking = false;
    }
    //Up left
    else if (dirangle>360-optpolartop){
      Serial1.print("RIGHT UP LEFT->");
      quadrant = 0;
      rightLeft = false;
      isTacking = false;
    }
    //bottom left
    else if (dirangle < 180 + optpolarbot && dirangle > 180){
      Serial1.print("RIGHT BOTTOM LEFT->");
      quadrant = 2;
      rightLeft = false;
      isTacking = false;
    }
    //bottom right
    else {
      Serial1.print("RIGHT BOTTOM RIGHT->");
      quadrant = 2;
      rightLeft = false;
      isTacking = false;
    }
  }
  //boat facing to left
  else{
    //Up right
    if (dirangle<optpolartop && dirangle>0){
      Serial1.print("LEFT UP RIGHT->");
      quadrant = 0;
      rightLeft = true;
      isTacking = false;
    }
    //Head directly to target to the right
    else if (dirangle>optpolartop && dirangle<180- optpolarbot){
      Serial1.print("LEFT DIRECT RIGHT->");
      //THIS IS WHERE WE WILL NEED TO CALL A TURN FUNCTION
//      start_box_time+=3000;
//      delay(3000);
      quadrant = 1;
      rightLeft = false;
      isTacking = false;
    }
    //Head directly to target to the left
    else if (dirangle < 180 + optpolarbot && dirangle > 180){
      Serial1.print("LEFT DIRECT LEFT->");
      quadrant = 1;
      rightLeft = true;
      isTacking = false;
    }
    //Up left
    else if (dirangle>360-optpolartop){
      Serial1.print("LEFT UP LEFT->");
      quadrant = 0;
      rightLeft = true;
      isTacking = false;
    }
    //bottom left
    else if (dirangle>optpolarbot + 180 && dirangle<360 -optpolartop){
      Serial1.print("LEFT BOTTOM LEFT->");
      quadrant = 2;
      rightLeft = true;
      isTacking = false;
    }
    //bottom right
    else {
      Serial1.print("LEFT BOTTOM RIGHT->");
      quadrant = 2;
      rightLeft = true;
      isTacking = false;
    }
  }
  //Call to nav function to set sail and tail angles
  nav(quadrant, rightLeft, windDir, anglewaypoint);
  //obstacle avoidance code
  if (avoid_test){
    avoidObject();
    if (avoiding){
      intended_angle=avoid_intended;
    }
  }

  // USE INTENDED ANGLE
  float offset = boatDirection-intended_angle;
  // Serial1.print("Intended angle: ");
  // Serial1.println(intended_angle);
  // Serial1.print("Offset from intended angle: ");
  // Serial1.println(offset);

  // Use the offset (a positive or negative angle that shows the amount that we
  // are facing away from the intended angle) and wind direction to set the sail
  // then offset the sail to the left or right w.r.t the tail to give it an angle
  // of attack
  tailAngle = windDir + offset;
  sailAngle = tailAngle + intended_angle_of_attack;

  tailAngle = (float)((int)tailAngle%360);
  tailAngle = tailAngle + 360;
  tailAngle = (float)((int)tailAngle%360);

  sailAngle = (float)((int)sailAngle%360);
  sailAngle = sailAngle+360;
  sailAngle = (float)((int)sailAngle%360);

  //Convert sail and tail from wrt north to wrt boat
  sailAngle = sailAngle - sensorData.boatDir;
  tailAngle = tailAngle - sensorData.boatDir;

  // convert sail to 0-360
  sailAngle = int(sailAngle+360)%360;

  //obstacle avoidance; still needs tuning
//  avoidObject();
//
//  Serial1.println("Angles w.r.t Boat after pixy correction:");
//  printSailTailSet();

  // convert tail to -180-180
  tailAngle = int(tailAngle+360)%360;
  if (tailAngle> 180) {tailAngle -= 360;}

  //Get servo commands from the calculated sail and tail angles
  if (sailAngle < 0) {
    sailAngle += 360;
  }

  // printSailTailSet();

  sensorData.sailAngleBoat = sailAngle;
  sensorData.tailAngleBoat = tailAngle;
  printSailTailSet();
// REAL BOAT SAIL AND TAIL MAPPING
//   tailAngle = tailMap(sailAngle, tailAngle);
//   sailAngle = sailMap(sailAngle);

   Serial1.println();

// TESTBENCH SAIL AND TAIL MAPPING
  tailAngle = tailMapBench(sailAngle, tailAngle);
  sailAngle = sailMapBench(sailAngle);

}
