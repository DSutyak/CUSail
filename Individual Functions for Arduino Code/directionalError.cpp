// #include <vector>

vector<double> directions;//contains readings of boat direction, must be updated with 400 after the waypoint is updated

// sketch of function: we have a global vector where each element is the directional error at a specific point in time
// more recent readings are towards the end of the vector, older readings towards the front
// if an entry is 400, it indicates that the boat switched waypoints
// this function returns the average directional error of the past 10 readings
// if there hasn't been 10 readings since the last waypoint, it will return however many readings have been made
// (i.e. some number less than 10)
double dirError() {
  // get the average of the past 10 readings
  double sum = 0;
  int count = 0;
  for(int i = directions.size()-1; i > directions.size()-11; i--) {
    count++;
    if(directions.get(i) == 400.0) // we don't want to incorporate values from previous waypoint
      break;
    sum += directions.get(i);
  }
  double avgDirError = sum / count;
  return avgDirError;
}


// this function updates the values of the directions vector
// takes as a parameter the expected direction (based on waypoint)
void diff(double expectedDir) {
    double diff = expectedDir - sensorData.boatDir;
    directions.push_back(diff);
}