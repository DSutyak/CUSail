/*Returns distance (in meters) between two global coordinates.
* Coordinates must be given in latitude and longitude */
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

  if (distance<30) {
  double longDiff = wayPoints[wpNum].longitude - sensorData.longi;
  double latDiff  = wayPoints[wpNum].latitude - sensorData.lati;
  distance = sqrt(pow(longDiff,2)+pow(latDiff,2));
  }

  return distance;
}
