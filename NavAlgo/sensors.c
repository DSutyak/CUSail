/* ************************************************************************** */
// Initialize and read from sensors
/* ************************************************************************** */

#include "plib.h" // peripheral library
#include "sensors.h"

data_t* sensorData;

/*sensor setup*/
void initSensors(void) {
  //Initialize data structure
  sensorData = (data_t*) malloc(sizeof(data_t));

  // TODO set Pin Modes

  //TODO Set Slave Select signals High i.e disable chips

  //TODO Initialize SPI

  sensorData->boat_direction = 0; //Boat direction w.r.t North
  sensorData->sailAngleBoat = 0; //Sail angle for use of finding wind wrt N
  sensorData->tailAngleBoat = 0; //Tail angle for use of finding wind wrt N
  sensorData->pitch = 0;
  sensorData->roll = 0;
  sensorData->wind_dir = 0; // Wind direction w.r.t North
  sensorData->wind_speed = 0;
  sensorData->x = 0; // Longitude of current global position;
  sensorData->y = 0; // Longitude of current global position;
  sensorData->lat=0;
  sensorData->longi=0;
}

/* TODO read from the IMU */
void readIMU(void) {
    // replace these with useful commands
    sensorData->boat_direction++;
    sensorData->pitch++;
    sensorData->roll++;
}

/* TODO read from anemometer */
void readAnemometer(void) {
    // replace these with useful commands
    sensorData->wind_dir++;
    sensorData->wind_speed++;
}

void readGPS(void) {
    sensorData->lat++;
    sensorData->longi++;
}
