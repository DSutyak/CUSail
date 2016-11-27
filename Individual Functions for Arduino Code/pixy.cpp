#include <SPI.h>  
#include <Pixy.h>
#include "mex.h"

Pixy pixy;

pixy.init();

angleofattack = 15;


//this code comes from https://forum.arduino.cc/index.php?topic=45626.0
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

Vector<double> xVals;

/** Updates xVals vector
*   0 to 1 indicates object on the starboard (closer to 1 = closer to edge of pixy cam view)
*  -1 to 0 indicates object on the port (closer to -1 = closer to edge of pixy cam view)
*   NOTE: This only detects objects set to
*   signature 1 on the pixy cam */
void getObjects() {
    uint16_t blocks = pixy.getBlocks();

    if (blocks) {
        for (int j = 0; j < blocks, j++) {
            if (pixy.blocks[j].signature == 1) {
                int32_t xLocation = pixy.blocks[j].x; // range: 0 to 319
                double half = xLocation / 2.0;
                xVals.push_back(xVals);
            }
        }
    }
}

void adjustDirection() {
    double courseChange;
    getObjects();
    int s = xVals.size();
    if (s > 1) {
        double initialReading = xVals.get(s-2);
        double recentReading = xVals.get(s-1);

        courseChange = initialReading - recentReading;
        if (Math.abs(courseChange) < 0.1) {
            //we need to make evasion manuerver
            if (recentReading > 0) {
                // we need to make a starboard turn
                angleofattack = 90.0*recentReading; // val from 0 to 90
                sailAngle=sensorData.windDir - angleofattack;
                tailAngle=sensorData.windDir;
            }
            else if (recentReading < 0) {
                // we need to make a port side turn
                angleofattack = (-1.0)*90.0*recentReading;
                sailAngle=sensorData.windDir + angleofattack;
                tailAngle=sensorData.windDir;
            }
        }
        else if (initialReading > recentReading) {
            // make a starboard turn proportional to recentReading
            angleofattack = Math.abs(90.0*recentReading); // val from 0 to 90
            sailAngle=sensorData.windDir - angleofattack;
            tailAngle=sensorData.windDir;
        }
        else if (initialReading < recentReading) {
             // we need to make a port side turn
            angleofattack = Math.abs(90.0*recentReading);
            sailAngle=sensorData.windDir + angleofattack;
            tailAngle=sensorData.windDir;
        }
    }
    
    sailAngle=sailAngle-sensorData.boatDir;
    tailAngle=tailAngle-sensorData.boatDir;
    sailAngle = int(sailAngle+360)%360;
    tailAngle = int(tailAngle+360)%360;
    if (tailAngle> 180) {tailAngle -= 360;}
    if (sailAngle < 0) {sailAngle += 360;}

}
