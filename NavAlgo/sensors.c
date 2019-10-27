/* ************************************************************************** */
// Initialize and read from sensors
/* ************************************************************************** */

#include "plib.h" // peripheral library
#include "sensors.h"
#include <xc.h>
#include "delay.h"

data_t* sensorData;

/*sensor setup*/
void initSensors(void) {
  //Initialize data structure
  sensorData = (data_t*) malloc(sizeof(data_t));

  // TODO set Pin Modes (for other communication protocols)

  //TODO Initialize SPI (Check this for clock rate)
  OpenSPI1(SPI_MODE8_ON|SPI_SMP_ON|MASTER_ENABLE_ON|SEC_PRESCAL_1_1|PRI_PRESCAL_1_1, SPI_ENABLE);
  // maybe configure interrupts here (could be useful)
  
  // initialize sensorData
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

// check this (SS, etc.)
uint8_t readSPI(uint8_t trans) {
    //TODO set SS low
    putcSPI1(trans);
    uint8_t result = getcSPI1();
    //TODO set SS high
    return result;
}

/* read from the IMU */
void readIMU(void) {
    uint8_t result = readSPI(0x01);
    result = readSPI(0xF6);
    delay_ms(1);
    
    //send command to read Euler angles
    result = readSPI(0x01);
    
    // get status from IMU
    result = readSPI(0xFF);
    while (result != 0x01) {
        delay_ms(1);
        result = readSPI(0xFF);
    }
    // read the pitch roll and yaw as a length 12 array of bytes
    uint8_t imu_data[12];
    int i;
    for (i = 0; i < 12; i++) {
        delay_ms(1);
        imu_data[i] = readSPI(0xFF);
    }
    //TODO convert data into usable values to update sensorData
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
