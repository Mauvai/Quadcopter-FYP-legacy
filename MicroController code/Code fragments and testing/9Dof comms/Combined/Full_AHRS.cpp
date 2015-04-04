/* 
Full Attitude and Heading Reference System - based on  sparkfun 9dof 
(obviously running for parallax propeller)

uses a second order complementary filter on the roll and picth, using gryo and
accelerometer data to feed it. 
*/

#include <stdio.h>            // Recommended over iostream for saving space
#include <propeller.h>        // Propeller-specific functions
#include "simpletools.h"
#include "dof_class.h"

        
//xyz structs containing accelerometer, Gyroscope and Magnetometer data
//They are decleared global so they cab be accessed anywhere  	
  	          



//function declarations

void run_dof(void *par);


//Globals
volatile float theta, phi, Heading, gyr_x, gyr_y, gyr_z;




int main()
{
  
  
  cog_run(&run_dof, 35); 
  /*
   *launches the dof_run function into a new core. 
   *Note that the int *(the stack size required) should be as low as possible,
   *while maintaining enough space for correct operation. (Space to run the function from)
   
   *The current value of 35 is close to as small as it can go (I have no idea how
   *it can go this low at all). If anything is added the function or class, this should be increased!
   
   *Note also that print operations cannot be run inside a new core, 
   *unless the terminal connection is first closed inside cog0, and then
   *reopened in  the new cog
   */
   
  
  
  int dt, t;            //stores timing values: t holds a clock value, dt is compared to it
  dt = CLKFREQ/2;    // 1Hz internal sampling  (dt = no of clock cycles in 1/2 seconds)
  
 

  while(1)
  {
    t = CNT;
    print("Theta; %.1f, Phi: %.1f, Head: %.1f, Gyx: %.1f, Gyy: %.1f, Gyz: %.1f\n", theta, phi, Heading, gyr_x, gyr_y, gyr_z);
    
    while(CNT-t < dt)
    {
      pause(1);
    }
      
    
  }
  
  
  
  


  return 0;
}




void run_dof(void *par)
{
  dof_class dof; //build a sensor object called dof 
  int dt, t;            //stores timing values: t holds a clock value, dt is compared to it
  dt = CLKFREQ/5;    // 2Hz internal sampling  (dt = no of clock cycles in 1/2 seconds)
  t = CNT;
  
  /* cant print without closing terminal access to core one if this is launched to a sperarate core 
  for (int i = 0; i < 100; i++)
  {
    dof.update();
  }   
  print("time per cycle: %f", (float)(CNT-t)/(float)CLKFREQ/(float)100); 
  print("\n CLKFREQ = %d\n", CLKFREQ);
  */
  
  
  while(1)
  {
    t = CNT;
    //count++;
    //print("\nCount: %d", count);
	  
   
   
   
    dof.update();
    theta = dof.theta;
    phi = dof.phi;
    Heading = dof.Heading;
    gyr_x = dof.Gyr_data.x;
    gyr_y = dof.Gyr_data.y;
    gyr_z = dof.Gyr_data.z;
    
    //print("\n%d, %d", CNT-t, dt);
    //CNT;
    while(CNT-t < dt)
    {
      //print("delayed");
      pause(1);
    }
  }
  
}