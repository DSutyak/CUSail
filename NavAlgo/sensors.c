/* ************************************************************************** */
// Initialize and read from sensors
/* ************************************************************************** */
#define _SUPPRESS_PLIB_WARNING // removes outdated plib warning
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING

#include <plib.h> // peripheral library
#include "sensors.h"
#include "servo.h"
#include "xc.h"
#include "delay.h"
#include <math.h>
#include "nmea/nmea.h"
#include <string.h>
#include "i2c_helper.h"


// system clock rate (as defined in config.h) is 40MHz
uint32_t clock_rate = 40000000;

float averageWeighting = 0.0625;
float prevSinWind = sin(270);
float prevCosWind = cos(270);
float prevWindDirection = 270;

// time of the last pulse on the hall effect sensor
int prevPulseMS = -1;
int prevPulseS = 0;
int prevPulseMin = 0;
int prevPulseHr = 0;

data_t* sensorData;


/*sensor setup*/
void initSensors(void) {    
    //Initialize data structure
    sensorData = (data_t*) malloc(sizeof(data_t));
    
    // TODO Initialize GPS (Check this)
    //OpenUART1(UART_EN | UART_NO_PAR_8BIT, UART_RX_ENABLE, 9600);
    
    /* Initialize Analog Inputs for Anemometer Pins (need 2 pins) */
    ANSELA = 0; ANSELB = 0; TRISA = 0xff; // set A as input
    
    CloseADC10();	// ensure the ADC is off before setting the configuration

	// define setup parameters for OpenADC10
	// Turn module on | ouput in integer | trigger mode auto | enable autosample
    // ADC_CLK_AUTO -- Internal counter ends sampling and starts conversion (Auto convert)
    // ADC_AUTO_SAMPLING_ON -- Sampling begins immediately after last conversion completes; SAMP bit is automatically set
    // ADC_AUTO_SAMPLING_OFF -- Sampling begins with AcquireADC10();
    #define PARAM1  ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_OFF //switch to ON for multiple

	// ADC ref external  | disable offset test | disable scan mode | do 1 sample | use single buf | alternate mode off
	#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_1 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF
	
    // use peripherial bus clock | set sample time | set ADC clock divider
    // ADC_CONV_CLK_Tcy2 means divide CLK_PB by 2 (max speed)
    // ADC_SAMPLE_TIME_5 seems to work with a source resistance < 1kohm
    #define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_5 | ADC_CONV_CLK_Tcy2 //ADC_SAMPLE_TIME_15| ADC_CONV_CLK_Tcy2

	// set AN11 as analog input
    #define PARAM4	ENABLE_AN11_ANA // pin 24 (RB13)

	// do not assign channels to scan
    #define PARAM5	SKIP_SCAN_AN0 | SKIP_SCAN_AN1 | SKIP_SCAN_AN2 | SKIP_SCAN_AN3 | SKIP_SCAN_AN4 | SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15
    
	// use ground as neg ref for A | use AN11 for input A     
	// configure to sample AN11
	SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN11 ); // configure to sample AN11
	OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using the parameters defined above
    
    EnableADC10(); // Enable the ADC
    
    // LiDAR and IMU (I2C)
    // Enable I2C1, BRG = (Fpb 40MHz / 2 / baudrate 300K) - 2.
	OpenI2C1( I2C_ON, 65);
  
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
    sensorData->msec = 0;
    sensorData->sec = 0;
    sensorData->min = 0;
    sensorData->hour = 0;
    
    // interrupt for hall effect sensor
    mINT0IntEnable(0);
    mINT0SetEdgeMode(1); // rising edge
    mINT0SetIntPriority(7);
    mINT0ClearIntFlag();
    mINT0IntEnable(1);
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

/* read from the IMU */
void readIMU(void) {
    char *data;
    data = i2c_read_imu();
    
    int i;
    int j;
    int k = 0;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 4; j++) {
            imu_data[i].b[j] = data[k];
            k++;
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

int mapInt(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
    return ((toHigh - toLow) * (value - fromLow)/(fromHigh-fromLow)) + toLow;
}

// read both wind direction and wind speed
void readAnemometer(void) {
    int adc_10 = ReadADC10(0); // wind direction?
    AcquireADC10();
    
    int angle = mapInt(adc_10, 0, 1023, 0, 360);
    angle = angle % 360;
    
    //get angle with respect to North
    float wind_wrtN = ((int)(angle + sensorData->sailAngleBoat))%360;
    wind_wrtN = ((int)(wind_wrtN + sensorData->boat_direction))%360;
    
    //filter wind
    float newSinWind = ( (sin(prevWindDirection*M_PI/180) + (averageWeighting)*sin(wind_wrtN*M_PI/180)) / (1 + averageWeighting) );
    float newCosWind = ( (cos(prevWindDirection*M_PI/180) + (averageWeighting)*cos(wind_wrtN*M_PI/180)) / (1 + averageWeighting) );
    wind_wrtN = atan2(newSinWind, newCosWind);
    wind_wrtN = wind_wrtN*180/M_PI;
    wind_wrtN = (wind_wrtN<0)?wind_wrtN+360:wind_wrtN;
    
    sensorData->wind_dir = wind_wrtN;
    prevWindDirection = wind_wrtN;
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
}

// TODO set pan and tilt as needed
float readLIDAR() {
    char *data;
    data = i2c_read_lidar();
    
    // distance in m
    float distance = (float)(data[0] * 256 + data[1])/100;
    
    //TODO do something with the distance/make a map
    return distance;
}

/* return the time in milliseconds */
long msTime(void) {
    return sensorData->msec + sensorData->sec * 1000 + sensorData->min * 60 * 1000 + sensorData->hour * 60 * 60 * 1000;
}

/* on a rising edge, count that a pulse occurred, approx wind speed */
void __ISR( _EXTERNAL_0_VECTOR, IPL7AUTO) INT0Interrupt(void) {
    if (prevPulseMS != -1) {
        long currentTime = msTime();
        long prevTime = prevPulseMS + prevPulseS * 1000 + prevPulseMin * 60 * 1000 + prevPulseHr * 60 * 60 * 1000;
    
        float deltaT = (currentTime - prevTime) / 1000.0; // time difference in seconds
    
        float speed = 2 * 2.25 / deltaT; // get time in mph
        speed *= 0.44704; // convert to m/s
    
        sensorData->wind_speed = speed;
    }
    
    // update the previous pulse
    prevPulseMS = sensorData->msec;
    prevPulseS = sensorData->sec;
    prevPulseMin = sensorData->min;
    prevPulseHr = sensorData->hour;
    
    mINT0ClearIntFlag();
}

// Timer for up time
void __ISR( _TIMER_1_VECTOR, IPL7AUTO) T1Interrupt(void) {
    sensorData->msec++;
    if (sensorData->msec >= 1000) {
        sensorData->msec = 0;
        sensorData->sec++;
    }
    if (sensorData->sec >= 60) {
        sensorData->sec = 0;
        sensorData->min++;
    }
    if (sensorData->min >= 60) {
        sensorData->min = 0;
        sensorData->hour++;
    }
    
    mT1ClearIntFlag();
}