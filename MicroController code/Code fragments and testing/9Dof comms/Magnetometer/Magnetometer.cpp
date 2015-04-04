/*
Magnetometer code, for HMC5883L digital compass (on a 9d0f sensor stick)
Assuming position, 51.8972° N, 8.4700° W (Cork)


*/

#include <stdio.h>            // Recommended over iostream for saving space
#include <propeller.h>        // Propeller-specific functions
#include "simpletools.h"

#define MagAddr 0x1E



i2c *DofBus;           //create pointer to i2c bus object
const int DofSCL = 2;  //devine SDA and SDL pins for i2c
const int DofSDA = 3;
const float pi = 3.14159265359;
//function declarations

inline int twos2signed(char MSB, char LSB);




int main()
{
  
  //create i2c bus. the 0 indicates pull up resistors are present on the i2c line
  DofBus = i2c_newbus(DofSCL, DofSDA, 0);
  pause(100);
  
  char ID[1];
  int val = i2c_in(DofBus, MagAddr, 0x0A, 1, (unsigned char*)ID, 1);
  //get device ID with i2c read
  print("ID readback: 0x%x\n\n", ID[0]);
  
  pause(100); //allow gyro clocks to stabalise


  //setup
  char reg_in[1];       //char to store single byte read in by I2C
  char reg_out[1];      //char to store single byte to be written to part with I2C
  reg_out[0] = 0x38; //2 sample averaging, 75Hx sample rate, normal measure mode (no self tiest bias)
  //reg_out[0] = 0x39; //apply self test bias
  i2c_out(DofBus, MagAddr, 0x00, 1, (unsigned char*)reg_out, 1);     
  i2c_in(DofBus, MagAddr, 0x00, 1, (unsigned char*)reg_in, 1);
  print("CRA: 0x%x (Self test mode)\n", reg_in[0]);

  reg_out[0] = 0x20; //Gain, default (1090 LSB per Gauss)
  i2c_out(DofBus, MagAddr, 0x01, 1, (unsigned char*)reg_out, 1);
  i2c_in(DofBus, MagAddr, 0x01, 1, (unsigned char*)reg_in, 1);
  print("CRB: 0x%x\n", reg_in[0]);

  //reg_out[0] = 0x00; //Continuous measure mode
  reg_out[0] = 0x01; //Single measure mode
  i2c_out(DofBus, MagAddr, 0x02, 1, (unsigned char*)reg_out, 1);
  i2c_in(DofBus, MagAddr, 0x02, 1, (unsigned char*)reg_in, 1);
  print("MR: 0x%x\n", reg_in[0]);

  pause(100); //allow gyro clocks to stabalise





   short x, y, z;          //Stores XYZ output after conversion from reister values
   float _x, _y, _z;
   char buffer[6];       //buffer which stores 56 register read backs from a multibyte i2c readback
   float angle;
   float RSS;
   
  while(1)
  {
      i2c_in(DofBus, MagAddr, 0x03, 1, (unsigned char*)buffer, 6);
      i2c_out(DofBus, MagAddr, 0x02, 1, (unsigned char*)reg_in, 1);
      x = ((buffer[4]<<8) | buffer[5])+150+72;
      y =  (-  ((buffer[0]<<8) | buffer[1]))-132-101;
      z = ((buffer[2]<<8) | buffer[3])+135+714;
      
       _x = x;
      _y = y;
      _z = z*1.17;
      
      angle = atan2(y, x)*180/pi;  //convert to degrees, add magnetic declination
      RSS = sqrt(x*x+y*y+z*z);
      
      print("\nX: %f  Y: %f  Z: %f, Heading: %f, RSS: %f", _x, _y, _z, angle , RSS);
      pause(500);
  }


  return 0;
}















inline int twos2signed(char MSB, char LSB)
{
   if (MSB >= 0x80)  //negative
   {
     return 0xFFFF0000 | (   ((MSB<<8) & 0xFF00)   |   LSB & 0xFF   );   
     //left pad from 16 bits to 32 bits with 1s
     //Do this because prop ints are 32 bits
     //assigning this value to a signed int results a negative value (as the leftmost bit is 1)
   }   
   return (   ((MSB<<8) & 0xFF00)   |   LSB & 0xFF   );
}  