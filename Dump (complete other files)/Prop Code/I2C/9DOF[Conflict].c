//https://propsideworkspace.googlecode.com/hg/Learn/Simple%20Libraries/Protocol/libsimplei2c/html/simplei2c_8h.html#a65bcbd826cf4a1dc5cf475cfd48b5526
//See above for commands
#include "simpletools.h"                      // Include simple tools
//#include <propeller.h>                        //Provides simple i2c start, stop, read, and write functions. See simpletools library for additional I2C functions.

i2c *Nine_DOF_Bus;

const int DofSCL = 0;
const int DofSDA = 1;

const int accAddr   = 0b1010011;                     //Ox53 (followed by read or write)corrosponding to OxA6 for write and OxA7 for read
const int magnAddr  = 0b0011110;                     //Ox1E (followed by read or write)corrosponding to Ox3C for write and Ox3D for read
const int gyrosAddr = 0b1101000;                     //Ox68 (followed by read or write)corrosponding to OxD0 for write and OxD1 for read


void Acc_setup(int Bus_ID, int Device_Addr)
{  

   unsigned char Read_back[] = {0,0} ; 
   i2c_in(Bus_ID , Device_Addr , 0x00 ,1 , (unsigned char*)Read_back ,2);   //Code to test the device ID. It should be 0xE5 or 0b11100101
   print("Accel ID is: %x \n",Read_back [0]);                            //Should Read back 0xE5 (But reads back 0, Cleary I am doing some thing stupid :(. )

   i2c_out(Bus_ID, Device_Addr, 0x2D ,1 , 0x00 ,2);                       // Write to Reg 0x2D so as to change the part from standy (0x00) to (0x08) measure mode.
   i2c_out(Bus_ID, Device_Addr, 0x2D ,1 , 8 ,2);                       // Write to Reg 0x2D so as to change the part from standy (0x00) to (0x08) measure mode.
   i2c_in(Bus_ID , Device_Addr, 0x2D ,1 , (unsigned char*)Read_back ,2);   // Print to serial to see if 0x08 was to reg 0x2D (REMOVE LATER)
   print("Read from reg 2d: %x \n", Read_back[0]);   
   

    i2c_out(Bus_ID, Device_Addr, 0x38 ,1 , 0x84 ,2);                      //Write to Reg 0x38 so as to chage FIFO to 'stream' and sets D2 to 1
    i2c_in(Bus_ID, Device_Addr, 0x38 ,1 , (unsigned char*)Read_back ,2);  // Print to serial to see if 0x84 was to reg 0x38 (REMOVE LATER)
    print("Read from reg 38: %x \n",Read_back[0]);  
    
}

void Mag_setup(int Bus_ID, int Device_Addr)
{
   unsigned char Read_back[] ={0,0}; 
   i2c_in(Bus_ID , Device_Addr , 10 ,1 , (unsigned char*)Read_back ,2);   //Code to test the device ID. It should be 0x48
   print("Mag ID is: %x \n",Read_back [0]);                               //Print to see if the regsistor is write was 0x48.
  
}  



int main()                                    // Main function
{
  
Nine_DOF_Bus = i2c_newbus(DofSCL, DofSDA, 1);  // Setup the Nine_DOF_I2C_Bus 
 
Acc_setup(Nine_DOF_Bus, accAddr);
Mag_setup(Nine_DOF_Bus, magnAddr);

unsigned char X[] = {0, 0, 0, 0, 0, 0, 0} ;


int temp;
  while(1)
  {
    X[2] = 4;
      i2c_in(Nine_DOF_Bus , accAddr , 0x32 ,1 , (unsigned char*)X ,7);
     int temp = ((X[1]<<8) & 0xFF00) | (X[0] & 0xFF);
/*      
      if (temp>= 32768)
      {
        temp = !temp;       //need to check this assigment
        temp = ++temp;        
        temp = -temp;
      }
      else
      {
        temp = temp;     // no need to change
      }                
 */     
      // to convert to 2 complemet check to see if the number is greater than or equal to 2^4 in this case as it is a 8 bit number.
      // Nor the function with 0XFF to get it into 1's complete if this is possible 
      // add one to the answer and as we have check to see if it is greater than 2^4 at the start we know to mulptply by one or not :). 
      print("\n X-axis %b %b \t", X[1], X[0]);
      print("Y-axis %b %b \t", X[3], X[2]);
      print("Z-axis %b %b \t", X[5], X[4]);
     print("\n concat X-axis %d \t", temp);
      pause(1000);
      
      

  }  
  
  return 0;
}




