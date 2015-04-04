#include <stdio.h>            // Recommended over iostream for saving space
#include <propeller.h>        // Propeller-specific functions
#include "simpletools.h"
#include "Data_types.h"
#include "dof_class.h"
#include "cordic.h"


char reg_in[1];       //char to store single byte read in by I2C
char reg_out[1];      //char to store single byte to be written to part with I2C
char buffer[6];       //buffer which stores 56 register read backs from a multibyte i2c readback

i2c *DofBus;           //create pointer to i2c bus object
const int DofSCL = 2;  //devine SDA and SDL pins for i2c
const int DofSDA = 3;


const int AccelAddr  = 0x53;
const int GyroAddr  = 0x68;
const int MagAddr  = 0x1E;

//accelerometer calibration - needs to be redone!
const int Bx = 10;
const int By = 4;
const int Bz = 0;

const float Sx = 1.0/263;  //perform division now, as division is lower than multiplication
const float Sy = 1.0/264;
const float Sz = 1.0/256;



//gryroscope calibration. needs to be redone!
const float xGyroConv =  1.0/ 14.375 / 1.065;
const float yGyroConv =  1.0/ 14.375 / 1.0;    //<- this is intentional
const float zGyroConv =  1.0/ 14.375 / 1.035;  // LSBs per (degree per second)
                                     


//At zero rotational velocity, the gyro outputs these values 
//(these are register values, not degrees per second)
//these values should be SUBTRACTED from the readback
const int xGyroDrift = -5;
const int yGyroDrift = -42;
const int zGyroDrift = -5;

const float pi = 3.14159265359;

const float degree2rad = pi/180;
const float rad2degree = 180/pi;
const float degree2rad_squared = degree2rad*degree2rad;

dof_class::dof_class()
{
	DofBus = i2c_newbus(DofSCL, DofSDA, 0);   //setup i2c bus to 9dof
	Accel_setup();   //setup 9dof chips
	Gyro_setup();
	Mag_setup();
	//print("\nSetup finished!\n\n");
	pause(1000);
}

void dof_class::update()
{
	read_accel();  //read the three dof chips
	read_gyro();
	read_mag();

  
	//calculate pitch and roll angles
	theta  = atan2f_cordic(-Acc_data.x, (sqrt(Acc_data.y*Acc_data.y + Acc_data.z*Acc_data.z)));    
	phi = atan2f_cordic(Acc_data.y, Acc_data.z);
	//based on the 1-2-3 rotations - see cillian's report


   /*   NON LINEARIED MAGNETOMETER FUNCTIONS - TRIG IS TOO SLOW AND NOT NEEDED
	mag_X = Mag_data.x * cosf(theta) + Mag_data.y * sinf(theta) * sinf(phi) + Mag_data.z * cosf(phi) * sinf(theta);  
	//note z is flipped to what the papper says, as our z axis is up, not down -  as we still want clockwise positive (looking down from top)
	mag_Y = Mag_data.y * cosf(phi) - Mag_data.z * sinf(phi);
   */

   //linearised  magnetometer functions    CURRENTLY BROKEN
   mag_X = Mag_data.x + Mag_data.y * theta * phi * degree2rad_squared + Mag_data.z * theta * degree2rad;  
	//note z is flipped to what the papper says, as our z axis is up, not down -  as we still want clockwise positive (looking down from top)
	mag_Y = Mag_data.y - Mag_data.z * phi * degree2rad;
   
   //note explicit call to float version of trig, i.e 'sinf'
	Heading = atan2f_cordic(mag_Y, mag_X);
	//see  "Applications of Magnetic Sensors for Low Cost Compass Systems"
   
}




void dof_class::read_accel()
{
	//read six bytes from the accelrometer, starting at reg 0x32
	//then convert to 2s compliment representation. 
	//note that registers are given LSB first
	//Then subtract bias, scale by sensitivity
	i2c_in(DofBus, AccelAddr, 0x32, 1, (unsigned char*)buffer, 6);
    
   //Convert to Gs (where 1g is 9.8ms^-2    (dont need to convert to Gs,but need to adjust for varying sensitivity)
   Acc_data.x = (twos2signed(buffer[1], buffer[0])-Bx)*Sx;
   Acc_data.y = (twos2signed(buffer[3], buffer[2])-By)*Sy;
   Acc_data.z = (twos2signed(buffer[5], buffer[4])-Bz)*Sz;
   
}

void dof_class::read_gyro()
{
	//see similar accelerometer function
    i2c_in(DofBus, GyroAddr, 0x1D, 1, (unsigned char*)buffer, 6);   //read 6 registers starting at 0x1D - data registers

    //convert 2s compliment split into 2 registers into signed and subtract gyro drift
    Gyr_data.x = (twos2signed(buffer[0], buffer[1]) - xGyroDrift)*xGyroConv;   
    Gyr_data.y = (twos2signed(buffer[2], buffer[3]) - yGyroDrift)*yGyroConv;    
    Gyr_data.z = (twos2signed(buffer[4], buffer[5]) - zGyroDrift)*zGyroConv;
}

void dof_class::read_mag()
{
	//see similar accelerometer function
	i2c_in(DofBus, MagAddr, 0x03, 1, (unsigned char*)buffer, 6);

    Mag_data.x = twos2signed(buffer[0], buffer[1]);
    Mag_data.y = twos2signed(buffer[4], buffer[5]);
    Mag_data.z = twos2signed(buffer[2], buffer[3]);
}



void dof_class::Accel_setup()
{   
    //set various registers to the required values to bring the chip into measure mode
    //the commented out code read the set registers to confirm they were written to.
    
	reg_out[0] = 0x08; //Power control: Measure mode
    i2c_out(DofBus, AccelAddr, 0x2D, 1, (unsigned char*)reg_out, 1);
    //i2c_in(DofBus, AccelAddr, 0x2D, 1, (unsigned char*)reg_in, 1);
    //print("Power control: 0x%x\n", reg_in[0]);
    
    reg_out[0] = 0x00;// bypass mode, no FIFO
    i2c_out(DofBus, AccelAddr, 0x38, 1, (unsigned char*)reg_out, 1);
    //i2c_in(DofBus, AccelAddr, 0x38, 1, (unsigned char*)reg_in, 1);
    //print("FIFO Setup: 0x%x\n", reg_in[0]);
    
    reg_out[0] = 0x00;// 10 bit mode, 2g range
    i2c_out(DofBus, AccelAddr, 0x31, 1, (unsigned char*)reg_out, 1);
    //i2c_in(DofBus, AccelAddr, 0x31, 1, (unsigned char*)reg_in, 1);
    //print("Data Format: 0x%x\n", reg_in[0]);
    
    reg_out[0] = 0x0B;// 200Hz sample rate
    i2c_out(DofBus, AccelAddr, 0x2C, 1, (unsigned char*)reg_out, 1);
    //i2c_in(DofBus, AccelAddr, 0x2C, 1, (unsigned char*)reg_in, 1);
    //print("Sample rate: 0x%x\n", reg_in[0]);
}

void dof_class::Gyro_setup()
{
	//see similar accelerometer function
	reg_out[0] = 0x01; //Power control: Set clk to xGyro, all gyros on
    i2c_out(DofBus, GyroAddr, 0x3E, 1, (unsigned char*)reg_out, 1);
    //i2c_in(DofBus, GyroAddr, 0x3E, 1, (unsigned char*)reg_in, 1);
    //print("Power control: 0x%x\n", reg_in[0]);
    
    reg_out[0] = 0x09; //Sample rate divider: set to 9, gives x/10 = sample output rate, where x is internal rate
  	 i2c_out(DofBus, GyroAddr, 0x15, 1, (unsigned char*)reg_out, 1);
    //i2c_in(DofBus, GyroAddr, 0x15, 1, (unsigned char*)reg_in, 1);
    //print("Rate divider: 0x%x\n", reg_in[0]);
    
    reg_out[0] = 0x1A; //scale selection and digital filter: bits 4&5: = 0b11: full scale. bits 2 10: 
    i2c_out(DofBus, GyroAddr, 0x16, 1, (unsigned char*)reg_out, 1);
    //i2c_in(DofBus, GyroAddr, 0x16, 1, (unsigned char*)reg_in, 1);
    //print("Digital Filter: 0x%x\n", reg_in[0]);
}

void dof_class::Mag_setup()
{
	//see similar accelerometer function
	reg_out[0] = 0x38; //2 sample averaging, 75Hx sample rate, normal measure mode (no self tiest bias)
    i2c_out(DofBus, MagAddr, 0x00, 1, (unsigned char*)reg_out, 1);     
    //i2c_in(DofBus, MagAddr, 0x00, 1, (unsigned char*)reg_in, 1);
    //print("CRA: 0x%x\n", reg_in[0]);
    
    reg_out[0] = 0x20; //Gain, default (1090 LSB per Gauss)
    i2c_out(DofBus, MagAddr, 0x01, 1, (unsigned char*)reg_out, 1);
    //i2c_in(DofBus, MagAddr, 0x01, 1, (unsigned char*)reg_in, 1);
    //print("CRB: 0x%x\n", reg_in[0]);
    
    reg_out[0] = 0x00; //Continuous measure mode
    i2c_out(DofBus, MagAddr, 0x02, 1, (unsigned char*)reg_out, 1);
    //i2c_in(DofBus, MagAddr, 0x02, 1, (unsigned char*)reg_in, 1);
    //print("MR: 0x%x\n", reg_in[0]);
}


inline int dof_class::twos2signed(char MSB, char LSB)
{
	//convert an 8bit MSB and  8bit LSB to a 32 bit signed int
   if (MSB >= 0x80)  //negative
   {
     return 0xFFFF0000 | (   ((MSB<<8) & 0xFF00)   |   LSB & 0xFF   );   
     //left pad from 16 bits to 32 bits with 1s
     //Do this because prop ints are 32 bits
     //assigning this value to a signed int results a negative value (as the leftmost bit is 1)
   }   
   return (   ((MSB<<8) & 0xFF00)   |   LSB & 0xFF   );
}  