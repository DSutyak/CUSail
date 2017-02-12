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

/*Changes the intended_angle_of_attack and intended_angle of the boat
given the sector the boat is in and the location of the next waypoint.
And then use the offset (a positive or negative angle that shows the 
amount that we are facing away from the intended angle) and wind direction
to set the sail then offset the sail to the left or right WRT the tail
to give it an angle of attack */
void set_boat_angle(float intended_angle, float intended_angle_of_attack){
// ATTEMPT TO USE INTENDED ANGLE
float offset=boatDirection-intended_angle;
tailAngle=windDir+offset;
sailAngle=tailAngle+intended_angle_of_attack;
}

void waypoint_reached(){
  printHitWaypointData();
  // turn = true;
  // firstIterTurn = true;
  wpNum += 1 ;

  //reset variables because we have reached the old waypoint
  r[0] = wayPoints[wpNum].longitude - sensorData.longi;
  r[1] = wayPoints[wpNum].latitude - sensorData.lati;
  w[0] = cos((sensorData.windDir)*(PI/180.0));
  w[1] = sin((sensorData.windDir)*(PI/180.0));
  coord_t currentPosition = {sensorData.lati, sensorData.longi};
  normr = havDist(wayPoints[wpNum], currentPosition);

      //Reset all stored boat directions to 0
  for (int i=0; i<numBoatDirReads; i++){
    boatDirections[i] = 0;
  }

  lightAllLEDs();
}





void obstacle_avoidance(){

  // This section of code implements avoidance manuever
  // if pixy detects an object in the boats path
  getObjects();
  int s = xVals.size();
  if (s > 1 && xVals[s-1] != 400.0 && xVals[s-2] != 400.0) {
   double initialReading = xVals[s-2];
   double recentReading = xVals[s-1];
   double courseChange = initialReading - recentReading;
   recentReading = (recentReading / 159.5) - 1.0; // this makes
   // recentReading from -1.0 to 1.0 with 0.0 being center of the frame
   if (Math.abs(courseChange) < 0.1) {
     // we need to make an evasion manuever
     if (recentReading > 0)
       recentReading = 1 - recentReading;// reverse recentReading measure
         // so that closer to 1 = closer to center
     else
       recentReading = -1 - recentReading;
     sailAngle += recentReading * 45;
     tailAngle += recentReading * 45;
   }
   else if (initialReading > recentReading) {
     // make starboard turn
     recentReading = Math.abs(45.0*recentReading);
     sailAngle += recentReading;
     tailAngle += recentReading;
   }
   else {
     // make port side turn
     recentReading = Math.abs(45.0*recentReading);
     sailAngle -= recentReading;
     tailAngle -= recentReading;
   }

  }

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

// converts an angle to a 0-360 range
float convert_to_360(float angle){
  angle=(float)((int)angle%360);
  angle=angle+360;
  angle=(float)((int)angle%360);
  return angle;
}