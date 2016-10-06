#include "mex.h"
#include <math.h>

/*          ----------NAVIGATION ALGORITHM----------

* Uses sensorData.windDir, sensorData.boatDir to set sailAngle and tailAngle. 
* sailAngle and tailAngle are set in terms of servo command numbers, but are first
* calculated in terms of angle w.r.t the boat direction

*/
void nShort() {
  
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

  //Reached waypoint!
  if((normr < detectionradius) && ((wpNum + 1) < numWP)){
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

  float dirangle=anglewaypoint-sensorData.windDir;
  //converts to 0-360
  dirangle=(float)((int)dirangle%360);
  dirangle=dirangle+360;
  dirangle=(float)((int)dirangle%360);

  float boatDirection = sensorData.boatDir; 
 
  boatDirection=360-(boatDirection-90);
  boatDirection=(float)((int)boatDirection%360);
  boatDirection=boatDirection+360;
  boatDirection=(float)((int)boatDirection%360);
  int side;

  // Wind w.r.t the boat
  float windboat;
  windboat=sensorData.windDir-boatDirection;

//write function that checks boat orientation and see if facing directly to waypoint, or if needs to turn

//  boat initially facing right
  if (sensorData.boatDir<180) {
    //Up right
    if (dirangle<optpolartop && dirangle>0){ 
      sailAngle=sensorData.windDir+angleofattack ;
      tailAngle=sensorData.windDir;
    }
    //Head directly to target to the right
    else if (dirangle>optpolartop && dirangle<180- optpolarbot){
      sailAngle=sensorData.windDir+angleofattack;
      tailAngle=sensorData.windDir;
    }
    //Head directly to target to the left
    else if (dirangle>optpolarbot + 180 && dirangle<360 -optpolartop){
      sailAngle=sensorData.windDir-angleofattack;
      tailAngle=sensorData.windDir;
    }
    //Up left
    else if (dirangle>360-optpolartop){
      sailAngle=sensorData.windDir+angleofattack;
      tailAngle=sensorData.windDir;
    }   
    //bottom left    
    else if (dirangle < 180 + optpolarbot && dirangle > 180){
      sailAngle=sensorData.windDir+angleofattack;
      tailAngle=sensorData.windDir;
    }
    //bottom right
    else {
      sailAngle=optpolarbot+angleofattack;
      tailAngle=optpolarbot;
    }
  }
  //boat facing to left  
  else{
    //Up right
    if (dirangle<optpolartop && dirangle>0){  
      sailAngle=sensorData.windDir-angleofattack ;
      tailAngle=sensorData.windDir;
    }
    //Head directly to target to the right
    else if (dirangle>optpolartop && dirangle<180- optpolarbot){
      sailAngle=sensorData.windDir+angleofattack;
      tailAngle=sensorData.windDir;
    }
    //Head directly to target to the left
    else if (dirangle>optpolarbot + 180 && dirangle<360 -optpolartop){
      sailAngle=sensorData.windDir-angleofattack;
      tailAngle=sensorData.windDir;
    }
    //Up left
    else if (dirangle>360-optpolartop){
      sailAngle=sensorData.windDir-angleofattack;
      tailAngle=sensorData.windDir;
    }   
    //bottom left    
    else if (dirangle < 180 + optpolarbot && dirangle > 180){
      sailAngle=sensorData.windDir-angleofattack;
      tailAngle=sensorData.windDir;
    }
    //bottom right
    else {
      sailAngle=sensorData.windDir - angleofattack;
      tailAngle=sensorData.windDir;
    }
  }

  //Convert sail and tail from wrt north to wrt boat
  sailAngle=sailAngle-sensorData.boatDir;
  tailAngle=tailAngle-sensorData.boatDir;
  sailAngle = int(sailAngle+360)%360;
  tailAngle = int(tailAngle+360)%360;
  if (tailAngle> 180) {tailAngle -= 360;}


  //Get servo commands from the calculated sail and tail angles
  if (sailAngle < 0) {
    sailAngle += 360;
  }

  tailAngle = tailMap(sailAngle, tailAngle);
  sailAngle = sailMap(sailAngle);
}