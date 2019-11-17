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
#include "nmea/nmea.h"
#include <string.h>
#include <peripheral/legacy/i2c_legacy.h>


// system clock rate (as defined in config.h)
uint32_t clock_rate = 40000000;

float angleCorrection = -26; //Big sail angle correction
float averageWeighting = 0.0625;

data_t* sensorData;

/*sensor setup*/
void initSensors(void) {
    //Initialize data structure
    sensorData = (data_t*) malloc(sizeof(data_t));

    // TODO set Pin Modes (for other communication protocols)
    
    // TODO Initialize GPS (Check this)
    OpenUART1(UART_EN | UART_NO_PAR_8BIT, UART_RX_ENABLE, 9600);
    
    
    //Analog stuff: params, initiating ADC
    CloseADC10();	// ensure the ADC is off before setting the configuration

	// define setup parameters for OpenADC10
				// Turn module on | output in integer | trigger mode auto | enable  autosample
	#define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

	// define setup parameters for OpenADC10
			    // ADC ref external    | disable offset test    | enable scan mode | perform 2 samples | use one buffer | use MUXA mode
       // note: to read X number of pins you must set ADC_SAMPLES_PER_INT_X
	#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_2 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF

	// define setup parameters for OpenADC10
	// 				  use ADC internal clock | set sample time
	#define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_15

	// define setup parameters for OpenADC10
				// set AN4 and AN5
	#define PARAM4	ENABLE_AN4_ANA | ENABLE_AN5_ANA

	// define setup parameters for OpenADC10
	// do not assign channels to scan
	#define PARAM5	SKIP_SCAN_AN0 | SKIP_SCAN_AN1 | SKIP_SCAN_AN2 | SKIP_SCAN_AN3 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN11 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15

	// use ground as neg ref for A 
	SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF); // use ground as the negative reference
	OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using parameter define above
	EnableADC10(); // Enable the ADC
    
    
    
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
    sensorData->x = 0; // X-coord of current global position;
    sensorData->y = 0; // Y-coord of current global position;
    sensorData->lat=0;
    sensorData->longi=0;
    
    date_t *dateTime;
    sensorData->dateTime = dateTime;
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
void readAnemometerDir(void) {
    
    while ( ! mAD1GetIntFlag() ) { }

    unsigned short int channel4;	// conversion result as read from result buffer
    int result = ReadADC10(0);
    
    int reading = ( (unsigned long) result)*360UL/16384UL;
    reading += angleCorrection;
    reading = (reading<0)?(reading+360):reading;
    
    float wind_wrtN = ((int)(reading + sensorData->sailAngleBoat))%360;
    wind_wrtN = ((int)(wind_wrtN + sensorData->boat_direction))%360;
    
    int prevWindDirection = 0; // FIX THIS

    //filter wind
    float newSinWind = ( (sin(prevWindDirection*M_PI/180) + (averageWeighting)*sin(wind_wrtN*M_PI/180)) / (1 + averageWeighting) );
    float newCosWind = ( (cos(prevWindDirection*M_PI/180) + (averageWeighting)*cos(wind_wrtN*M_PI/180)) / (1 + averageWeighting) );
    wind_wrtN = atan2(newSinWind, newCosWind);
    wind_wrtN = wind_wrtN*180/M_PI;
    wind_wrtN = (wind_wrtN<0)?wind_wrtN+360:wind_wrtN;
    // replace these with useful commands
    sensorData->wind_dir =  wind_wrtN;
    
}



void readAnemometerSpeed(void) {
    
    while ( ! mAD1GetIntFlag() ) { }
    unsigned short int channel5;	// conversion result as read from result buffer
    int result = ReadADC10(0); //Get a value between 0 and 1023 from the analog pin connected to the anemometer
    
    int reading = ( (unsigned long) result)*360UL/16384UL;
    
    float voltageConversionConstant = .004882814; //This constant maps the value provided from the analog read function, which ranges from 0 to 1023, to actual voltage, which ranges from 0V to 5V
    //^^^ is that right
    
    int sensorDelay = 1000; //Delay between sensor readings, measured in milliseconds (ms)
    float voltageMin = .4; // Mininum output voltage from anemometer in mV.
    float windSpeedMin = 0; // Wind speed in meters/sec corresponding to minimum voltage

    float voltageMax = 2.0; // Maximum output voltage from anemometer in mV.
    float windSpeedMax = 32; // Wind speed in meters/sec corresponding to maximum voltage
  
    float sensorVoltage = result * voltageConversionConstant; //Convert sensor value to actual voltage

    //Convert voltage value to wind speed using range of max and min voltages and wind speed for the anemometer
    if (sensorVoltage <= voltageMin){
        sensorData->wind_dir =  0;
    }else {
          sensorData->wind_dir = (sensorVoltage - voltageMin)*windSpeedMax/(voltageMax - voltageMin); //For voltages above minimum value, use the linear relationship to calculate wind speed.
    }
    
 }

void readGPS(void) {
    int count = -1;
    char buffer[80];
    while (DataRdyUART1()) {
        buffer[count++] = ReadUART1();
    }
    nmeaINFO *info;
    nmeaPARSER *parser;

    nmea_zero_INFO(info);
    nmea_parser_init(parser);
    
    nmea_parse(parser, buffer, (int)strlen(buffer), info);
    nmea_parser_destroy(parser);
    
    sensorData->lat = info->lat;
    sensorData->longi = info->lon;
    sensorData->dateTime->year = info->utc.year;
    sensorData->dateTime->month = info->utc.mon;
    sensorData->dateTime->day = info->utc.day;
    sensorData->dateTime->hour = info->utc.hour;
    sensorData->dateTime->minute = info->utc.min;
    sensorData->dateTime->seconds = info->utc.sec;
}

void readI2C(){
    uint8_t i2cbyte1
    uint8_t i2cbyte2
    char i2cData[0]
    int DataSz 4
    
    OpenI2C1( I2C_ON, 400);
    StartI2C1();
    
    
    int Index = 0;
    while(dataSz)
        {
        MasterWriteI2C1( i2cData[Index++] );
        IdleI2C1();//Wait to complete
        DataSz--;
        if( I2C1STATbits.ACKSTAT )
           break;
    }    
    
    RestartI2C1();//Send the Stop condition
    IdleI2C1();//Wait to complete
    
    MasterWriteI2C1( (SlaveAddress << 1) | 1 ); //transmit read command
    IdleI2C1();//Wait to complete
    
    unsigned char i2cbyte = MasterReadI2C1();
    StopI2C1();
    IdleI2C1();
    
    
} 
  
