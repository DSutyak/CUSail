// I2C Library for LiDAR and IMU


#define USE_AND_OR	// To enable AND_OR mask setting for I2C.
//#include <i2c.h>
#include "plib.h"
#include "tft_master.h"

#define SLAVE_ADDRESS_LIDAR 0x66
#define DIST_LOC 0x00

#define SLAVE_ADDRESS_IMU 0xEE
#define IMU_WRITE 0x42
#define IMU_EULER 0x01
#define IMU_READ 0x43

// Wait by executing nops
void i2c_wait(unsigned int cnt)
{
	while(--cnt)
	{
		asm( "nop" );
		asm( "nop" );
	}
}

// Read a char from the register specified by address
char* i2c_read_lidar()
{
    char i2c_header[2];
    i2c_header[0] = ( SLAVE_ADDRESS_LIDAR | 0 );	//device address & WR
	i2c_header[1] = DIST_LOC;                //register address

    StartI2C1();	//Send the Start Bit
	IdleI2C1();		//Wait to complete

    int i;
	for(i = 0; i < 2; i++)
	{
        MasterWriteI2C1( i2c_header[i] );
		IdleI2C1();		//Wait to complete

		//ACKSTAT is 0 when slave acknowledge,
		//if 1 then slave has not acknowledge the data.
		if( I2C1STATbits.ACKSTAT )
        {
			break;
        }
	}

    //now send a start sequence again
	RestartI2C1();	//Send the Restart condition
	i2c_wait(10);
	//wait for this bit to go back to zero
	IdleI2C1();	//Wait to complete

	MasterWriteI2C1( SLAVE_ADDRESS_LIDAR | 1 ); //transmit read command
	IdleI2C1();		//Wait to complete

	// read 2 bytes back
    static char data[2];
    data[0] = MasterReadI2C1();
	IdleI2C1();	//Wait to complete
    data[1] = MasterReadI2C1();
	IdleI2C1();	//Wait to complete

    StopI2C1();	//Send the Stop condition
	IdleI2C1();	//Wait to complete

    return data;
}


// Read a char from the register specified by address
char* i2c_read_imu()
{
    char i2c_header[4];
    i2c_header[0] = ( SLAVE_ADDRESS_LIDAR | 0 );	//device address & WR
	i2c_header[1] = IMU_WRITE;
    i2c_header[2] = IMU_EULER;
    i2c_header[3] = IMU_READ;

    StartI2C1();	//Send the Start Bit
	IdleI2C1();		//Wait to complete

    int i;
	for(i = 0; i < 4; i++)
	{
        MasterWriteI2C1( i2c_header[i] );
		IdleI2C1();		//Wait to complete

		//ACKSTAT is 0 when slave acknowledge,
		//if 1 then slave has not acknowledge the data.
		if( I2C1STATbits.ACKSTAT )
        {
			break;
        }
	}

    //now send a start sequence again
	RestartI2C1();	//Send the Restart condition
	i2c_wait(10);
	//wait for this bit to go back to zero
	IdleI2C1();	//Wait to complete

	MasterWriteI2C1( SLAVE_ADDRESS_IMU | 1 ); //transmit read command
	IdleI2C1();		//Wait to complete

	// read 12 bytes back
    static char data[12];
	for(i = 0; i < 12; i++) {
		data[i] = MasterReadI2C1();
		IdleI2C1();	//Wait to complete
	}

    StopI2C1();	//Send the Stop condition
	IdleI2C1();	//Wait to complete

    return data;
}
