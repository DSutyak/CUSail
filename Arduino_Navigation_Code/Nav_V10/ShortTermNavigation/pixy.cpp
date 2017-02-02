#include <Arduino.h>
#include <math.h>

// this class allows us to use a vector data structure within Arduino code
// Take from https://forum.arduino.cc/index.php?topic=45626.0
template<typename Data>
class Vector {
  size_t d_size; // Stores no. of actually stored objects
  size_t d_capacity; // Stores allocated capacity
  Data *d_data; // Stores data
  public:
    Vector() : d_size(0), d_capacity(0), d_data(0) {}; // Default constructor
    Vector(Vector const &other) : d_size(other.d_size), d_capacity(other.d_capacity), d_data(0) { d_data = (Data *)malloc(d_capacity*sizeof(Data));
        memcpy(d_data, other.d_data, d_size*sizeof(Data)); }; // Copy constuctor
    ~Vector() { free(d_data); }; // Destructor
    Vector &operator=(Vector const &other) { free(d_data); d_size = other.d_size; d_capacity = other.d_capacity;
        d_data = (Data *)malloc(d_capacity*sizeof(Data));
        memcpy(d_data, other.d_data, d_size*sizeof(Data));
        return *this; }; // Needed for memory management
    void push_back(Data const &x) { if (d_capacity == d_size) resize(); d_data[d_size++] = x; }; // Adds new value. If needed, allocates more space
    size_t size() const { return d_size; }; // Size getter
    Data const &operator[](size_t idx) const { return d_data[idx]; }; // Const getter
    Data &operator[](size_t idx) { return d_data[idx]; }; // Changeable getter
  private:
    void resize() { d_capacity = d_capacity ? d_capacity*2 : 1; Data *newdata = (Data *)malloc(d_capacity*sizeof(Data)); memcpy(newdata, d_data, d_size * sizeof(Data)); free(d_data); d_data = newdata; };// Allocates double the old space
};

/*----------PixyCam Variables----------*/
Vector<double> xVals;

  // This section of code implements avoidance manuever
  // if pixy detects an object in the boats path
 //   getObjects();
  // int s = xVals.size();
  // if (s > 1 && xVals.get(s-1) != 400.0 && xVals.get(s-2) != 400.0) {
  //  double initialReading = xVals.get(s-2);
  //  double recentReading = xVals.get(s-1);
  //  double courseChange = initialReading - recentReading;
  //  recentReading = (recentReading / 159.5) - 1.0; // this makes
  //  // recentReading from -1.0 to 1.0 with 0.0 being center of the frame
  //  if (Math.abs(courseChange) < 0.1) {
  //    // we need to make an evasion manuever
  //    if (recentReading > 0)
  //      recentReading = 1 - recentReading;// reverse recentReading measure
  //        // so that closer to 1 = closer to center
  //    else
  //      recentReading = -1 - recentReading;
  //    sailAngle += recentReading * 45;
  //    tailAngle += recentReading * 45;
  //  }
  //  else if (initialReading > recentReading) {
  //    // make starboard turn
  //    recentReading = Math.abs(45.0*recentReading);
  //    sailAngle += recentReading;
  //    tailAngle += recentReading;
  //  }
  //  else {
  //    // make port side turn
  //    recentReading = Math.abs(45.0*recentReading);
  //    sailAngle -= recentReading;
  //    tailAngle -= recentReading;
  //  }

  // }

  // //////////
  // PIXY CAM STUFF TO GO HERE
  // /////////

  /** Updates xVals vector
*   Only updates x-position
*   NOTE: This only detects objects set to
*   signature 1 on the pixy cam */
//void getObjects() {
//    uint16_t blocks = pixy.getBlocks();
//    if (blocks) {
//        for (int j = 0; j < blocks, j++) {
//            if (pixy.blocks[j].signature == 1) {
//                int32_t xLocation = pixy.blocks[j].x; // range: 0 to 319
//                xVals.push_back(xLocation);
//            }
//        }
//    }
//    else {
//        xVals.push_back(400.0); // no objects were detected
//    }
//}
