/* ************************************************************************** */
// Initialize and read from sensors
/* ************************************************************************** */
#define _SUPPRESS_PLIB_WARNING // removes outdated plib warning
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING

#include "plib.h" // peripheral library
#include "sensors.h"
#include "xc.h"
#include "delay.h"
#include <math.h>

data_t* sensorData;

/*sensor setup*/
void initSensors(void) {
    //Initialize data structure
    sensorData = (data_t*) malloc(sizeof(data_t));

    // TODO set Pin Modes (for other communication protocols)

    //TODO Initialize SPI (Check this for clock rate)
    OpenSPI1(SPI_MODE8_ON|SPI_SMP_ON|MASTER_ENABLE_ON|SEC_PRESCAL_2_1|PRI_PRESCAL_4_1, SPI_ENABLE);
    ANSELBbits.ANSB3 = 0; // enable RB3 (pin 7) as digital for SS
    TRISBbits.TRISB3 = 0; // enable RB3 (pin 7) as output for SS
    PORTBbits.RB3 = 1; // set SS high (active low)
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

union u_types {
    int8_t b[4];
    float fval;
} imu_data[3];  // Create 3 unions, one for each Euler angle

// swap IMU input data endianness
void endianSwap(int8_t temp[4]) {
  int8_t myTemp = temp[0];
  temp[0] = temp[3];
  temp[3] = myTemp;
  myTemp = temp[1];
  temp[1] = temp[2];
  temp[2] = myTemp;
}

// check this (SS, etc.)
uint8_t readSPI(uint8_t trans) {
    PORTBbits.RB3 = 0; // set SS low
    delay_ms(1);
    putcSPI1(trans);
    uint8_t result = getcSPI1();
    delay_ms(1);
    PORTBbits.RB3 = 1; // set SS high
    return result;
}

/* read from the IMU */
void readIMU(void) {
    uint8_t result = readSPI(0x01);
    result = readSPI(0xF6);
    delay_ms(1);
    
    //send command to read Euler angles
    result = readSPI(0x01);
    
    // get status from IMU, wait until ready to transfer data
    result = readSPI(0xFF);
    while (result != 0x01) {
        delay_ms(1);
        result = readSPI(0xFF);
    }
    
    // when ready, read the pitch roll and yaw as a length 12 array of bytes
    int i;
    int j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 4; j++) {
            imu_data[i].b[j] = readSPI(0xFF);
            delay_ms(1);
        }
    }
    
    // convert data into usable values to update sensorData
    int m;
    for(m = 0; m < 3; m++) {
        endianSwap(imu_data[m].b);
    }
    
    // update sensorData
    sensorData->boat_direction =  ((imu_data[1].fval)*(180/M_PI));
    if (sensorData->boat_direction < 0) {
        sensorData->boat_direction += 360;
    }
    sensorData->pitch  = (imu_data[0].fval)*(180/M_PI);
    sensorData->roll = (imu_data[2].fval)*(180/M_PI);
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
