
//Basic gyro control code
//includes a hyperterminal menu to: 
   //Read back dev ID
   //set up necessary registers
   //read back rotational velocity data
   //calibrate zero level bias
   //calculate a cumulative angle over 10 seconds (allows scaling factor tolerance calculation)
   
//Required circuit: Prop and 9Dof. 9 dof i2c bus connects to pins 2 and 3. 
//9DoF requires a seperate 5V power rail. must run from simple IDE with terminal(hyperterminal should work too?)




#include <stdio.h>            // Recommended over iostream for saving space
#include <propeller.h>        // Propeller-specific functions
#include "simpletools.h"

#define GyroAddr 0x68
//scaling factor - a register readback is multiplied by this much to get a degrees/second value
//the correction at the end is related to the "scale factor tolerance"
const float xGyroConv =  1.0/ 14.375 / 1.065;
const float yGyroConv =  1.0/ 14.375 / 1.0;    //<- this is intentional
const float zGyroConv =  1.0/ 14.375 / 1.035;  // LSBs per (degree per second)
                                     //hyperterminal case 7 tests indicate that the 
                                     //quoted scaling factor is 3.5% too high
                                     
//At zero rotational velocity, the gyro outputs these values 
//(these a register values, not degrees per second)
//these values should be SUBTRACTED from the readback
const int xGyroDrift = -5;
const int yGyroDrift = -42;
const int zGyroDrift = -5;


i2c *DofBus;           //create pointer to i2c bus object
const int DofSCL = 2;  //devine SDA and SDL pins for i2c
const int DofSDA = 3;

//function declarations
void hyperterminal(void);
inline int twos2signed(char MSB, char LSB);


int main()
{
  
  //create i2c bus. the 0 indicates pull up resistors are present on the i2c line
  DofBus = i2c_newbus(DofSCL, DofSDA, 0);
  
  
  unsigned char ID = 2;
  int val = i2c_in(DofBus, GyroAddr, 0x00, 1, (unsigned char*)ID, 1);
  //get device ID with i2c read
  
  print("ID readback: 0x%x", ID);
  
  char temp[5];
  pause(100); //allow gyro clocks to stabalise

  //start hyperterminal menu
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
  int x, y, z;          //Stores XYZ output after conversion from reister values
  int dt, t;            //stores timing values: t holds a clock value, dt is compared to it
  char buffer[6];       //buffer which stores 56 register read backs from a multibyte i2c readback
  
  

  int tic = CNT; 
  float temp = 0;
       
  float sum = 0;
  
  float xsum = 0;
  float ysum = 0;
  float zsum = 0;
  
  float prev = 0;
  float current = 0;
  




	
  while(1)
	{
    //menu options
	 print("\n\nSelect a menu option with a number, followed by enter\n");
    print("1. Exit hyperterminal \n");
    print("2. Read back ID register\n");
    print("3. Setup: set clock to Xgyro, sample rate div to 9 (100Hz output), filter 98Hz\n");
    print("4. Stream 5 seconds of data\n");
    print("5. Stream 30 seconds of data \n");
    print("6. 2 second bias calculation\n");
    print("7. 10 second cumulative angle calculation\n");
    print("8. Timing readout debugger\n");
    
    //get user input. Must be a number! follow with enter
    scanf("%d", &menu_no);

    switch(menu_no)
    {
      
      case 1:
        return;  //Exit hyperterminal
        break; 
        
        
    	case 2:    //dev ID readback
    	i2c_in(DofBus, GyroAddr, 0x00, 1, (unsigned char*)reg_in, 1);  
      print("ID readback: 0x%x\n\n", reg_in[0]);
      break;

      case 3: //see ITG3200 data sheet
       	reg_out[0] = 0x01; //Power control: Set clk to xGyro, all gyros on
  		  i2c_out(DofBus, GyroAddr, 0x3E, 1, (unsigned char*)reg_out, 1);
        i2c_in(DofBus, GyroAddr, 0x3E, 1, (unsigned char*)reg_in, 1);
      	print("Power control: 0x%x\n", reg_in[0]);

    		reg_out[0] = 0x09; //Sample rate divider: set to 9, gives x/10 = sample output rate, where x is internal rate
  			i2c_out(DofBus, GyroAddr, 0x15, 1, (unsigned char*)reg_out, 1);
        i2c_in(DofBus, GyroAddr, 0x15, 1, (unsigned char*)reg_in, 1);
        print("Rate divider: 0x%x\n", reg_in[0]);

        reg_out[0] = 0x1A; //scale selection and digital filter: bits 4&5: = 0b11: full scale. bits 2 10: 
    	  i2c_out(DofBus, GyroAddr, 0x16, 1, (unsigned char*)reg_out, 1);
        i2c_in(DofBus, GyroAddr, 0x16, 1, (unsigned char*)reg_in, 1);
        print("Digital Filter: 0x%x\n", reg_in[0]);

    		break;
    		


  		case 4: 
        dt = CLKFREQ*5;    // 5 second timeout
        //CLKFREQ contains the number of ticks in one second, CNT returns current tick no
        t = CNT;           // Mark current time by storing in t
        while(CNT - t < dt)    // Repeat until timeout
        {
    		  i2c_in(DofBus, GyroAddr, 0x1D, 1, (unsigned char*)buffer, 6);   //read 6 registers starting at 0x1D - data registers
          x = twos2signed(buffer[0], buffer[1]) - xGyroDrift;    //convert 2s compliment split into 2 registers into signed
          y = twos2signed(buffer[2], buffer[3]) - yGyroDrift;    //and subtract gyro drift
          z = twos2signed(buffer[4], buffer[5]) - zGyroDrift;
          print("\nX: %d  Y: %d  Z: %d\n", x, y, z); 
          pause(500);
          
        }
    		break;

      case 5: 
        dt = CLKFREQ*30;    // 30 second timeout
        t = CNT;           // Mark current time by storing in t
        while(CNT - t < dt)    // Repeat until timeout
        {
          i2c_in(DofBus, GyroAddr, 0x1D, 1, (unsigned char*)buffer, 6);
          x = twos2signed(buffer[0], buffer[1]) - xGyroDrift;
          y = twos2signed(buffer[2], buffer[3]) - yGyroDrift;
          z = twos2signed(buffer[4], buffer[5]) - zGyroDrift;
          print("\nX: %d  Y: %d  Z: %d\n", x, y, z);
          pause(500);
        }
        break;
      
      
      
        
      case 6:
        tic = CNT; 
        dt = CLKFREQ/100;   //100HZ cycle  NB. integer math 
        xsum = 0;
        ysum = 0;
        zsum = 0;
       
        
        for (int i = 0; i < 400; i++) //  4 seconds
        {
          t = CNT; // Mark current time by storing in t
          i2c_in(DofBus, GyroAddr, 0x1D, 1, (unsigned char*)buffer, 6);
          xsum += ((float)twos2signed(buffer[0], buffer[1])) / 400;   //running 400 piece average
          ysum += ((float)twos2signed(buffer[2], buffer[3])) / 400;
          zsum += ((float)twos2signed(buffer[4], buffer[5])) / 400;
        
          while(CNT - t < dt){}
        }                   
        
      
        
              
        
        print("\nelapsed time: %f\n", ((float)(CNT-tic))/CLKFREQ );
        print("xdrift: %f, ydrift: %f, zdrift: %f\n", xsum, ysum, zsum);
        break;
        
        
        
        
      case 7:
        //currently set up for y axis
      
        //by taking the variance over a known total rotation (ie rotating exactly 90 degrees, and reading 93 degrees)
        //a variance in the  Gyro scaling factor can be estimated as (93-90)/90 = 3.33%
        //print("\nGyro conversion factor: %f\n", GyroConv);
        dt = CLKFREQ/100;   //200HZ cycle  NB integer math here
        prev = 0;
        current = 0;
        sum = 0;
                
        //aqquire first sample
        t = CNT; // Mark current time by storing in t
        i2c_in(DofBus, GyroAddr, 0x1D, 1, (unsigned char*)buffer, 6);
        prev= (    (float)twos2signed(buffer[2], buffer[3])   -yGyroDrift) * yGyroConv;    
        while(CNT - t < dt);
        
        //calculate angle over time by taking consecutive values, and using tustins approximation to integration (check name?)
        //sum this angle over consecutive points
        for (int i = 0; i < 1000; i++) //  1000 samples/100HZ = 10 seconds.
        {
          t = CNT; // Mark current time by storing in t
          i2c_in(DofBus, GyroAddr, 0x1D, 1, (unsigned char*)buffer, 6);
          current= (    (float)twos2signed(buffer[2], buffer[3])   -yGyroDrift) * yGyroConv;


          sum +=     (current +prev) * 0.01/2;    //Tustins approximation to integration
          
          prev = current;

          while(CNT - t < dt){}
        }          
        print("\nFinal Rotation: %f\n", sum);

        break;
        
        
        
        
        
        
        
        
        
        
        
        
        
      case 8:
        //debugging case - replace code with whatever suits
        
        print("\n\nCLKFREQ: %d\n", CLKFREQ);
        tic = CNT; 
        dt = CLKFREQ/2;   //2HZ cycle  NB. integer math 
        
        
        sum = 0;
        
        for (int i = 0; i < 10; i++) //  5 seconds
        {
          t = CNT; // Mark current time by storing in t
          pause(1);
          //print("Reading: %f      sum: %f\n", temp, sum);
          
          while(CNT - t < dt){}
        }          
        
       
              
        print("\nCNT-tic: %d\n", CNT-tic);
        print("elapsed time: %f\n", ((float)(CNT-tic))/CLKFREQ );
        
        
        
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


