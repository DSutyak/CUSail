// I2C Library provided by 
// http://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/f2015/dc686_nn233_hz263/final_project_webpage_v2/dc686_nn233_hz263/dc686_nn233_hz263/i2c_helper.h


#define USE_AND_OR	// To enable AND_OR mask setting for I2C. 
//#include <i2c.h>
#include "plib.h"
#include "tft_master.h"
#define SLAVE_ADDRESS 0x66
#define DIST_LOC 0x00

// Wait by executing nops
void i2c_wait(unsigned int cnt)
{
	while(--cnt)
	{
		asm( "nop" );
		asm( "nop" );
	}
}

// Write a number of chars from data specified by num to the specified address
void i2c_write(char address, char *data, int num)
{
    char i2c_header[2];
    i2c_header[0] = SLAVE_ADDRESS | 0;	//device address & WR
	i2c_header[1] = address;            //register address

    StartI2C1();	//Send the Start Bit
	IdleI2C1();		//Wait to complete

    int i;
	for(i = 0; i < num + 2; i++)
	{
        if(i < 2)
            MasterWriteI2C1( i2c_header[i] );
        else
            MasterWriteI2C1( data[i - 2] );
		IdleI2C1();		//Wait to complete

		//ACKSTAT is 0 when slave acknowledge, 
		//if 1 then slave has not acknowledge the data.
		if( I2C1STATbits.ACKSTAT )
			break;
	}
    
    StopI2C1();	//Send the Stop condition
	IdleI2C1();	//Wait to complete
}

// Read a char from the register specified by address
char* i2c_read(char address)
{
    char i2c_header[2];
    i2c_header[0] = ( SLAVE_ADDRESS | 0 );	//device address & WR
	i2c_header[1] = address;                //register address

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

	MasterWriteI2C1( SLAVE_ADDRESS | 1 ); //transmit read command
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