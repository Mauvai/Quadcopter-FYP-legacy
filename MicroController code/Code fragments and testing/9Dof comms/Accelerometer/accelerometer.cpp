//calibration link
//      http://www.kionix.com/sites/default/files/AN012%20Accelerometer%20Errors.pdf

#include <stdio.h>            // Recommended over iostream for saving space
#include <propeller.h>        // Propeller-specific functions
#include "simpletools.h"

#define AccelAddr 0x53

i2c *DofBus;
const int DofSCL = 15;
const int DofSDA = 14;

//staraight line sensor
const int Bx = 19;
const int By = 4;
const int Bz = 0;

const float Sx = 1.0/263;  //perform division now, as division is lower than multiplication
const float Sy = 1.0/264;
const float Sz = 1.0/256;

/*
//right angle sensor
const int Bx = 9.75;
const int By = 7.9;
const int Bz = 0;

const float Sx = 1.0/265.25;
const float Sy = 1.0/266.75;
const float Sz = 1.0/256.0;
*/


void hyperterminal(void);
inline int twos2signed(char MSB, char LSB);

int main()
{
  
  
  DofBus = i2c_newbus(DofSCL, DofSDA, 0);
  
  
  unsigned char ID = 2;
  int val = i2c_in(DofBus, AccelAddr, 0x00, 1, (unsigned char*)ID, 1);
  print("Mark\n");
  print("ID readback: 0x%x", ID);
  
  char temp[5];
  
  hyperterminal();
  
  
  while(1)
  {
      
  }


  return 0;
}



void hyperterminal(void)
{
  int menu_no;          //hyperterminal switch variable
  char reg_in[1];       //char to store single byte read in by I2C
  char reg_out[1];      //char to store single byte to be written to part with I2C
  float x, y, z;          //Stores XYZ output after conversion from reister values
  int dt, t;            //stores timing values: t holds a clock value, dt is compared to it
  char buffer[6];       //buffer which stores 56 register read backs from a multibyte i2c readback

  float sum[] = {0, 0, 0};
  
  while(1)
  {
    	print("\n\nSelect a menu option with a number, followed by enter\n");
    	print("1. Exit hyperterminal \n");
      print("2. Read back ID register\n");
    	print("3. Config device (measure mode, fifo off, 10 bit 2g range, 200Hz\n");   
    	print("4. Read XYZ data\n");
      print("5. Stream 10 seconds of data \n");
      print("6. 1 second average reading (100 samples)");
    	

    	scanf("%d", &menu_no);
      print("\n%d\n", menu_no);
     
    	switch(menu_no)
    	{

        case 1:
          return;
          break;

    		case 2:
    			i2c_in(DofBus, AccelAddr, 0x00, 1, (unsigned char*)reg_in, 1);
          print("ID readback: 0x%x\n\n", reg_in[0]);
    			break;
    		case 3:
          reg_out[0] = 0x08; //Power control: Measure mode
    			i2c_out(DofBus, AccelAddr, 0x2D, 1, (unsigned char*)reg_out, 1);
          i2c_in(DofBus, AccelAddr, 0x2D, 1, (unsigned char*)reg_in, 1);
          print("Power control: 0x%x\n", reg_in[0]);
          
          reg_out[0] = 0x00;// bypass mode, no FIFO
    			i2c_out(DofBus, AccelAddr, 0x38, 1, (unsigned char*)reg_out, 1);
          i2c_in(DofBus, AccelAddr, 0x38, 1, (unsigned char*)reg_in, 1);
          print("FIFO Setup: 0x%x\n", reg_in[0]);
          
          reg_out[0] = 0x00;// 10 bit mode, 2g range
    			i2c_out(DofBus, AccelAddr, 0x31, 1, (unsigned char*)reg_out, 1);
          i2c_in(DofBus, AccelAddr, 0x31, 1, (unsigned char*)reg_in, 1);
          print("Data Format: 0x%x\n", reg_in[0]);
          
          
          
          reg_out[0] = 0x0B;// 200Hz sample rate
    			i2c_out(DofBus, AccelAddr, 0x2C, 1, (unsigned char*)reg_out, 1);
          i2c_in(DofBus, AccelAddr, 0x2C, 1, (unsigned char*)reg_in, 1);
          print("Data Format: 0x%x\n", reg_in[0]);
          
    			break;
    		
    		case 4:
      print("\n%f\n",Sx);
    			i2c_in(DofBus, AccelAddr, 0x32, 1, (unsigned char*)buffer, 6);
           x = (twos2signed(buffer[1], buffer[0])-Bx)*Sx;
           y = (twos2signed(buffer[3], buffer[2])-By)*Sy;
           z = (twos2signed(buffer[5], buffer[4])-Bz)*Sz;
          
          print("\nX: %f  Y: %f  Z: %f\n", x, y, z); 
       
    			break;
        case 5:
          
          dt = CLKFREQ/100;    // 100Hz internal sampling
          
          for (int j = 0; j<30; j++)
          {
            t = CNT;           // Mark current time by storing in t
            x = 0;
            y = 0;
            z = 0;
            for (int i = 0; i < 100; i++)
            {          
              i2c_in(DofBus, AccelAddr, 0x32, 1, (unsigned char*)buffer, 6);
              x += (twos2signed(buffer[1], buffer[0])-Bx)*Sx*0.01;
              y += (twos2signed(buffer[3], buffer[2])-By)*Sy*0.01;
              z += (twos2signed(buffer[5], buffer[4])-Bz)*Sz*0.01;
              while(CNT - t < dt);    // Repeat until timeout
            }
            print("\nX: %f  Y: %f  Z: %f\n", x, y, z);
          }           
          break;

        case 6: 
          
          dt = CLKFREQ/100;    // 10 second timeout
          t = CNT;           // Mark current time by storing in t
          sum[0] = 0;
          sum[1] = 0;
          sum[2] = 0;
          for (int i = 0; i< 100; i++)    // Repeat until timeout
          {
            i2c_in(DofBus, AccelAddr, 0x32, 1, (unsigned char*)buffer, 6);
            sum[0] += twos2signed(buffer[1], buffer[0]) * 0.01;
            sum[1] += twos2signed(buffer[3], buffer[2]) * 0.01;
            sum[2] += twos2signed(buffer[5], buffer[4]) * 0.01;
            //print("\nX: %d  Y: %d  Z: %d\n", x, y, z); 
            while(CNT-t<dt);
          }
          print("\nx: %f, y: %f, z: %f\n\n", sum[0], sum[1], sum[2]);
          
          break;
    		
    		default:
    			//code
    			break;
    	}
   }     
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