//returns how far of course the boat is in kilometers
double distanceError(coord_t currPos) {
  // current coordiantes
  double xa = currPos.longitude;
  double ya = sensorData.latitude;


  // final destination
  double x1 = wayPoints[maxPossibleWaypoints].longitude; // wayPoints[wpNum].longitude
  double y1 = wayPoints[maxPossibleWaypoints].latitude;

  // initial coordinates
  double x0;
  double y0;

  coord_t i0;
  i0.latitude = y0;
  i0.longitude = x0;

  coord_t i1;
  i1.latitude = y1;
  i1.longitude = x1;
  
  // We first need to caclulate where the boat should be
  
  double vx; // this is the expected velocity that is constant, 
    // can be calculated initially from wind speed, wind direction, and waypoint position
  double tf = havDist(i0, i1) / vx; // calulate the final time when we expect the boat to arrive at the waypoint

  double tq; // current time (should be collected with current coordinates)

  double xq; // the expected x coordinate
  xq = x0 + vx / tq; // expected x location
  // implement a simple linear function to denote the expected path of the boat
  yq = (xq - x0) * ( (y1 - y0) / (x1 - x0) ) + y0;

  coord_t iq;
  iq.latitude = yq;
  iq.longitude = xq;

  return havDist(iq, ia);
}
