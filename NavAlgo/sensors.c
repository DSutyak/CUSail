/* ************************************************************************** */
// Initialize and read from sensors
/* ************************************************************************** */
#define _SUPPRESS_PLIB_WARNING // removes outdated plib warning
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING

#include <plib.h> // peripheral library
#include "sensors.h"
#include "servo.h"
#include "xc.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
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

// UART1 for GPS
char data[82];
int rxIdx = 0;

data_t* sensorData;

//testing
int numPulses = 0;

/*sensor setup*/
void initSensors(void) {    
    //Initialize data structure
    sensorData = (data_t*) malloc(sizeof(data_t));
    
    //Initialize GPS
    // define setup Configuration 1 for OpenUARTx
		// Module Enable 
		// Work in IDLE mode 
		// Communication through usual pins 
		// Disable wake-up 
		// Loop back disabled 
		// Input to Capture module from ICx pin 
		// no parity 8 bit 
		// 1 stop bit 
		// IRDA encoder and decoder disabled 
		// CTS and RTS pins are disabled 
		// UxRX idle state is '1' 
		// 16x baud clock - normal speed
	#define config1 	UART_EN | UART_IDLE_CON | UART_RX_TX | UART_DIS_WAKE | UART_DIS_LOOPBACK | UART_DIS_ABAUD | UART_NO_PAR_8BIT | UART_1STOPBIT | UART_IRDA_DIS | UART_DIS_BCLK_CTS_RTS| UART_NORMAL_RX | UART_BRGH_SIXTEEN
	
	// define setup Configuration 2 for OpenUARTx
		// IrDA encoded UxTX idle state is '0'
		// Enable UxRX pin
		// Disable UxTX pin
		// Interrupt on transfer of every character to TSR 
		// Interrupt on every char received
		// Disable 9-bit address detect
		// Rx Buffer Over run status bit clear
	#define config2		UART_TX_PIN_LOW | UART_RX_ENABLE | UART_TX_DISABLE | UART_INT_TX | UART_INT_RX_CHAR | UART_ADR_DETECT_DIS | UART_RX_OVERRUN_CLEAR

	// Open UART1 with config1 and config2
	OpenUART1( config1, config2, clock_rate/16/9600-1);
		
	// Configure UART1 RX Interrupt with priority 2
    IFS1bits.U1EIF = 0;
    IFS1bits.U1RXIF = 0;
    IFS1bits.U1TXIF = 0;
    IPC8bits.U1IP = 2;
    IPC8bits.U1IS = 2;
    IEC1bits.U1RXIE = 1;
    IEC1bits.U1TXIE = 0;
    
	//ConfigIntUART1(UART_INT_PR2 | UART_RX_INT_EN);
    PPSInput(3, U1RX, RPA4);
    
    /* Initialize Analog Inputs for Anemometer Pins (need 2 pins) */
    ANSELA = 0; ANSELB = 0; // set A as input
    
// the ADC ///////////////////////////////////////
    // configure and enable the ADC
	CloseADC10();	// ensure the ADC is off before setting the configuration

	// define setup parameters for OpenADC10
	// Turn module on | ouput in integer | trigger mode auto | enable autosample
    // ADC_CLK_AUTO -- Internal counter ends sampling and starts conversion (Auto convert)
    // ADC_AUTO_SAMPLING_ON -- Sampling begins immediately after last conversion completes; SAMP bit is automatically set
    // ADC_AUTO_SAMPLING_OFF -- Sampling begins with AcquireADC10();
    #define PARAM1  ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON //

	// define setup parameters for OpenADC10
	// ADC ref external  | disable offset test | disable scan mode | do 1 sample | use single buf | alternate mode off
	#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_2 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF
        //
	// Define setup parameters for OpenADC10
    // use peripherial bus clock | set sample time | set ADC clock divider
    // ADC_CONV_CLK_Tcy2 means divide CLK_PB by 2 (max speed)
    // ADC_SAMPLE_TIME_5 seems to work with a source resistance < 1kohm
    #define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_15 | ADC_CONV_CLK_Tcy 

	// define setup parameters for OpenADC10
	// set AN11 and  as analog inputs
	#define PARAM4	ENABLE_AN11_ANA | ENABLE_AN5_ANA // 

	// define setup parameters for OpenADC10
    // DO not skip the channels you want to scan
    // do not specify channels  5 and 11
	#define PARAM5	SKIP_SCAN_AN0 | SKIP_SCAN_AN1 | SKIP_SCAN_AN2 | SKIP_SCAN_AN3 | SKIP_SCAN_AN4 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15

	// use ground as neg ref for A 
    // actual channel number is specified by the scan list
    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF); // 
	OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using the parameters defined above

	EnableADC10(); // Enable the ADC
  ///////////////////////////////////////////////////////

    
    // LiDAR and IMU (I2C)
    // Enable I2C1, BRG = (Fpb 40MHz / 2 / baudrate 300K) - 2.
	OpenI2C1( I2C_ON, 0x0C2); //65 gives 300K, this is 9600
  
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
    unsigned char *data;
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
    sensorData->boat_direction = ((imu_data[1].fval)*(180/M_PI));
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
    int adc_10 = ReadADC10(0);
    
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

int readEncoder(void) {
    return ReadADC10(1);
}

void checkSentence() {
    static char lat[20];
    static char longi[20];
    
    int latIdx = 0;
    int longIdx = 0;
    
    int east = -1;
    int north = 1;
    
    int valid = 0;
    
    // only parse GPGLL sentences (for now)
    if (data[0] == '$' && data[1] == 'G' && data[2] == 'P' && data[3] == 'G' && 
            data[4] == 'L' && data[5] == 'L') {
        int fieldNum = 0; // keep track of how many fields have been parsed
        int i = 6;
        int done = 0;
        
        while (i < 82 && done == 0) {
            i++;
            if (data[i] == ',') {
                fieldNum++;
            } else if (fieldNum == 0) {
                lat[latIdx] = data[i];
                latIdx++;
            } else if (fieldNum == 1) {
                north = (data[i] == 'N') ? 1 : -1;
            } else if (fieldNum == 2) {
                longi[longIdx] = data[i];
                longIdx++;
            } else if (fieldNum == 3) {
                east = (data[i] == 'E') ? 1 : -1;
            } else if (fieldNum == 5 && data[i] == 'A') {
                valid = 1;
            }
        }
        
        if (valid) {
            char lat_deg[2], longi_deg[3], lat_min[10], longi_min[10];
            
            lat_deg[0] = lat[0];
            lat_deg[1] = lat[1];
            longi_deg[0] = longi[0];
            longi_deg[1] = longi[1];
            longi_deg[2] = longi[2];
            int idx = 0;
            while (idx < 10) {
                lat_min[idx] = lat[idx+2];
                longi_min[idx] = longi[idx+3];
                idx++;
            }
            
            double lat_dd, longi_dd, lat_mm, longi_mm;
            lat_dd = atof(lat_deg);
            longi_dd = atof(longi_deg);
            lat_mm = atof(lat_min);
            longi_mm = atof(longi_min);
            
            sensorData->lat = north * (lat_dd + lat_mm/60.0);
            sensorData->longi = east * (longi_dd + longi_mm/60.0);
        }
    }
}

float readLIDAR() {
    // average three measurements to reduce finickiness
    unsigned char *data1;
    data1 = i2c_read_lidar();
    
    unsigned char *data2;
    data2 = i2c_read_lidar();
    
    unsigned char *data3;
    data3 = i2c_read_lidar();
    
    // distance in m
    float distance1 = (float)(data1[0] * 256 + data1[1])/100;
    float distance2 = (float)(data2[0] * 256 + data2[1])/100;
    float distance3 = (float)(data3[0] * 256 + data3[1])/100;
    
    float distance = (distance1 + distance2 + distance3) / 3;
    
    return distance;
}

/* return the time in milliseconds */
long msTime(void) {
    return sensorData->msec + sensorData->sec * 1000 + sensorData->min * 60 * 1000 + sensorData->hour * 60 * 60 * 1000;
}

/* on a rising edge, count that a pulse occurred, approx wind speed */
void __ISR( _EXTERNAL_0_VECTOR, IPL7AUTO) INT0Interrupt(void) {
    long currentTime = msTime();
    long prevTime = prevPulseMS + prevPulseS * 1000 + prevPulseMin * 60 * 1000 + prevPulseHr * 60 * 60 * 1000;
    int thresshold = 10; // simulate filter for noise
    
    if (prevPulseMS != -1 && currentTime - prevTime > thresshold) {
        numPulses++; // testing purposes
            
        float deltaT = (currentTime - prevTime) / 1000.0; // time difference in seconds
    
        float speed = 2 * 2.25 / deltaT; // get time in mph
        speed *= 0.44704; // convert to m/s
    
        sensorData->wind_speed = speed;
    }
    
    if (prevPulseMS == -1 || currentTime - prevTime > thresshold) {
        // update the previous pulse
        prevPulseMS = sensorData->msec;
        prevPulseS = sensorData->sec;
        prevPulseMin = sensorData->min;
        prevPulseHr = sensorData->hour;
    }
    
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

// UART 1 interrupt handler
// it is set at priority level 2
void __ISR(_UART1_VECTOR, IPL2AUTO) IntUart1Handler(void)
{
	// Is this an RX interrupt?
	if(IFS1bits.U1RXIF)
	{
        while (DataRdyUART1()) {
            char in = (char) ReadUART1();
        
            if (in == '$') {
                rxIdx = 0;
                data[rxIdx] = in;
            } else if (rxIdx > 0 && (in == '\r' || in == '\n' || in == '*')) {
                checkSentence();
                rxIdx = 82;
            } else if (rxIdx < 82) {
                rxIdx++;
                data[rxIdx] = in;
            }
        }
		
        // Clear the RX interrupt Flag
	    IFS1bits.U1RXIF = 0;
	}

	// We don't care about TX interrupt
	if (IFS1bits.U1TXIF)
	{
		IFS1bits.U1TXIF = 0;
	}
}
