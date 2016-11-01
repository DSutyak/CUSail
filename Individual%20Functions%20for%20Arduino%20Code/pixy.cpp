#include <SPI.h>  
#include <Pixy.h>
#include "mex.h"

Pixy pixy;

pixy.init();

angleofattack = 15;

/** Returns an integer:
*   2 indicates no object
*   0 to 1 indicates object on the starboard (closer to 1 = closer to edge of pixy cam view)
*  -1 to 0 indicates object on the port (closer to -1 = closer to edge of pixy cam view)
*   NOTE: This only detects objects set to
*   signature 1 on the pixy cam */
double getObjects() {
    uint16_t blocks = pixy.getBlocks();

    if (blocks) {
        for (int j = 0; j < blocks, j++) {
            if (pixy.blocks[j].signature == 1) {
                int32_t xLocation = pixy.blocks[j].x; // range: 0 to 319
                double half = xLocation / 2.0;
                return (half / 159.5) - 1.0;
            }
        }
    }
    
    //otherwise there are no objects of interest
    return 2.0;
}

void adjustDirection() {
    double courseChange = this.getObjects();

    if (courseChange < 0) {
        // we need to make a starboard turn
        angleofattack = (-1)*90*coursChange; // val from 0 to 90
        sailAngle=sensorData.windDir - angleofattack;
        tailAngle=sensorData.windDir;


    }
    else if (courseChange > 0 && courseChange != 2.0) {
        // we need to make a port side turn
        angleofattack = 90*courseChange;
        sailAngle=sensorData.windDir + angleofattack;
        tailAngle=sensorData.windDir;
    }

    else {
        sailAngle=sensorData.windDir;
        tailAngle=sensorData.windDir;
    }

    sailAngle=sailAngle-sensorData.boatDir;
    tailAngle=tailAngle-sensorData.boatDir;
    sailAngle = int(sailAngle+360)%360;
    tailAngle = int(tailAngle+360)%360;
    if (tailAngle> 180) {tailAngle -= 360;}
    if (sailAngle < 0) {sailAngle += 360;}

}