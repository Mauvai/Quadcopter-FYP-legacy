//https://propsideworkspace.googlecode.com/hg/Learn/Simple%20Libraries/Protocol/libsimplei2c/html/simplei2c_8h.html#a65bcbd826cf4a1dc5cf475cfd48b5526
//See above for commands
#include "simpletools.h"                      // Include simple tools



#define  DofSCL  0
#define  DofSDA  1
#define accAddr    0x53                     //Ox53 (followed by read or write)corrosponding to OxA6 for write and OxA7 for read
#define magnAddr   0x1E                     //Ox1E (followed by read or write)corrosponding to Ox3C for write and Ox3D for read
#define gyrosAddr  0x68                     //Ox68 (followed by read or write)corrosponding to OxD0 for write and OxD1 for read
#define Mag_scale  2.56                     // See page 13 of data sheet (note could put in a check statement but would take up to much space :( )
# define angel_of_dec - 4.216                 // -4^{0} 13^{'} West
#define  angel_of_dec_red  0.073583
#define pie 3.14
int convert_2_sgn(unsigned char Reg_MSB , unsigned char Reg_LSB);
void Acc_setup(i2c* Bus_ID);
void Mag_setup(i2c* Bus_ID);
i2c *Nine_DOF_Bus;

int main()                                    // Main function
{
  
Nine_DOF_Bus = i2c_newbus(DofSCL, DofSDA, 1);  // Setup the Nine_DOF_I2C_Bus 
 
Acc_setup(Nine_DOF_Bus);
Mag_setup(Nine_DOF_Bus);

unsigned char Acc_output[6]  ;
unsigned char Mag_output[6]  ;
 int acc_array[3];
 int mag_array[3];
 int  mag_array_Raw[3];
 float direction =7;
 float  atan1 =7.00011;
 float mag;
 float sum;
 float sum_xy;
 float mag_xy;
 float theat;
  while(1)
  {

      i2c_in(Nine_DOF_Bus , accAddr , 0x32 ,1 , (unsigned char*)Acc_output ,6);
      i2c_in(Nine_DOF_Bus , magnAddr , 0x03 ,1 , (unsigned char*)Mag_output ,6);



  for (int i =0 ; i <6; i += 2)
  {
    acc_array[i/2] = convert_2_sgn(Acc_output[i+1], Acc_output[i]);
    mag_array_Raw[i/2] = convert_2_sgn(Mag_output[i], Mag_output[i+1]);
    mag_array[i/2] = Mag_scale * convert_2_sgn(Mag_output[i], Mag_output[i+1]);
    
  }   
  

    
    
    
        
      
      if ((mag_array[2]==0) && (mag_array[0] >= 0))
        {
          direction = 360;
        }         
        else if ((mag_array[2]>0) && (mag_array[0] > 0))
          {
           direction = atan((float)mag_array[2] / (float)mag_array[0]);
          }            
        else if ((mag_array[2]>0) && (mag_array[0] == 0))
          {
            direction = PI/2;
          }
        else if ((mag_array[2]>0) && (mag_array[0] < 0))
          {
            direction = PI + atan((float)mag_array[2] / (float)mag_array[0]);
          }           
        else if ((mag_array[2]== 0) && (mag_array[0] < 0))
          {
            direction = PI;
          }  
   
        else if ((mag_array[2]<0) && (mag_array[0] < 0))
          {
            direction = PI + atan((float)mag_array[2] / (float)mag_array[0]);
          }  
        else if ((mag_array[2]<0) && (mag_array[0] == 0))
          {
            direction = 3* PI /2;
          }     
         else 
          {
            direction = 2*PI + atan((float)mag_array[2] / (float)mag_array[0]);
          }     
                                                  
 
 
 
 
 
  if (direction < angel_of_dec_red)
  {
   direction = 2* PI - angel_of_dec_red;
  }   
   else
   {
   direction = direction - angel_of_dec_red; 
   }    
      atan1 =  direction *(180/PI);
        
       sum= mag_array[2]*mag_array[2] +mag_array[1]*mag_array[1] + mag_array[0]*mag_array[0];
     mag = sqrt((float)sum); 
             
           mag_xy =  mag_array[2]*mag_array[2] +   mag_array[0]*mag_array[0];         
                       
                       
      print("\n\n Acceleromemter  X axis: %d, Y axis: %d, Z axis: %d  \t",  acc_array[0],  acc_array[1], acc_array[2]);
      print("\n Mag  X axis: %d, Y axis: %d, Z axis: %d  \t",  mag_array[0],  mag_array[2], mag_array[1]);
      print("\n Mag raw data X axis: %d, Y axis: %d, Z axis: %d  \t",  mag_array_Raw[0],  mag_array_Raw[2], mag_array_Raw[1]);
      print("\n Direction of device (Rads): %f  \t", direction);
      print("\n Direction of device (Degrees)%f",atan1);
      //print("\n Mag %f", mag);
 /*
     int X_axis_acc =  convert_2_sgn(Acc_output[1], Acc_output[0]);
     int Y_axis_acc =  convert_2_sgn(Acc_output[3], Acc_output[2]);
     int Z_axis_acc =  convert_2_sgn(Acc_output[5], Acc_output[4]);
     
     int X_axis_mag = convert_2_sgn(Mag_output[0], Mag_output[1]);
     int Z_axis_mag = convert_2_sgn(Mag_output[2], Mag_output[3]);
     int Y_axis_mag = convert_2_sgn(Mag_output[4], Mag_output[5]);
     
     print("\n\n Acceleromemter  X axis: %d, Y axis: %d, Z axis: %d  \t",  X_axis_acc,  Y_axis_acc , Z_axis_acc );
     print("\n Mag  X axis: %d, Y axis: %d, Z axis: %d  \t",  X_axis_mag,  Y_axis_mag , Z_axis_mag);
   
 */  



      pause(2000);
      
      

  }  
  
  return 0;
}




int convert_2_sgn(unsigned char Reg_MSB , unsigned char Reg_LSB)
{
      int Concatenate_array = ((Reg_MSB <<8) & 0xFF00) | (Reg_LSB & 0xFF);
     
      if (Concatenate_array >= 0x8000)
      {
        Concatenate_array ^= 0xFFFF;              //This converts 2 complement to decimal (Breaks my code at the moment :( )
        Concatenate_array = ++Concatenate_array;        
        Concatenate_array = -Concatenate_array;
      }
     

return  Concatenate_array ;      
  
}  



void Acc_setup(i2c* Bus_ID)
{  
   unsigned char reg_input[1];
   unsigned char Read_back[1]; 
   i2c_in(Bus_ID , accAddr , 0x00 ,1 , (unsigned char*)Read_back ,1);   //Code to test the device ID. It should be 0xE5 or 0b11100101
   print("Accel ID is: %x \n",Read_back [0]);                            //Should Read back 0xE5 (But reads back 0, Cleary I am doing some thing stupid :(. )
   
   reg_input[0]=0x0B;
   i2c_out(Bus_ID, accAddr, 0x2C ,1 , (unsigned char*)reg_input ,1);   // Write to Reg 0x2C so as to change the part (0x0B) to sampling of 200 Hz.
   i2c_in(Bus_ID , accAddr, 0x2C ,1 , (unsigned char*)Read_back ,1);   // Print to serial to see if 0x0B was to reg 0x2C (REMOVE LATER)
   print("Read from reg 2c: %x \n", Read_back[0]);   

   reg_input[0]=0x08;
   i2c_out(Bus_ID, accAddr, 0x2D ,1 , (unsigned char*)reg_input ,1);   // Write to Reg 0x2D so as to change the part from standy (0x00) to (0x08) measure mode.
   i2c_in(Bus_ID , accAddr, 0x2D ,1 , (unsigned char*)Read_back ,1);   // Print to serial to see if 0x08 was to reg 0x2D (REMOVE LATER)
   print("Read from reg 2d: %x \n", Read_back[0]);   
   
    reg_input[0]=0x84;
    i2c_out(Bus_ID, accAddr, 0x38 ,1 , (unsigned char*)reg_input ,1); //Write to Reg 0x38 so as to chage FIFO to 'stream' and sets D2 to 1
    i2c_in(Bus_ID, accAddr, 0x38 ,1 , (unsigned char*)Read_back ,1);  // Print to serial to see if 0x84 was to reg 0x38 (REMOVE LATER)
   print("Read from reg 38: %x \n",Read_back[0]);  
    
}

void Mag_setup(i2c* Bus_ID)
{ 
   unsigned char reg_input[1];
   unsigned char Read_back[1]; 
   i2c_in(Bus_ID , magnAddr , 10 ,1 , (unsigned char*)Read_back ,1);   //Code to test the device ID. It should be 0x48
   print("Mag ID is: %x \n",(unsigned char*) Read_back [0]);                               //Print to see if the regsistor is write was 0x48.
  
   reg_input[0]= 0x78;
   i2c_out(Bus_ID, magnAddr, 0x00 ,1 , (unsigned char*)reg_input,1);                      //Write to Reg 0x00 so as to set the number of averaging to 8 samples per measurement output and also to change the Data Output Rata to 75 Hz
   i2c_in(Bus_ID , magnAddr , 0x00 ,1 , (unsigned char*)Read_back ,1); 
   print("Reg 0x00 is: %x \n",Read_back [0]);
   
  reg_input[0]= 0xA0; 
  i2c_out(Bus_ID, magnAddr, 0x01 ,1 , (unsigned char*)reg_input,1);                      //write to Reg 0x01 so as to set gain (LSB/Gauss) of the device (Note if this is changed so to must Mag_scale (see define staements above)) 
  i2c_in(Bus_ID , magnAddr , 0x01 ,1 , (unsigned char*)Read_back ,1); 
  print("Reg 0x01 is: %x \n",Read_back [0]);
  
  reg_input[0]= 0x00;
  i2c_out(Bus_ID, magnAddr, 0x02 ,1 , (unsigned char*)reg_input,1);                      //write to Reg 0x02 so as to set the mode to continues-measurement mode
  i2c_in(Bus_ID , magnAddr , 0x02 ,1 , (unsigned char*)Read_back ,1); 
   print("Reg 0x02 is: %x \n",Read_back [0]);
  
}  

