/**
* This is the main Blank Simple C++ Project program file.
*/

#include <stdio.h>            // Recommended over iostream for saving space
#include <propeller.h>        // Propeller-specific functions
#include "dof_class.h"
#include "efficient_math.h"

extern float pi = 3.14159265359;
inline int twos2signed(char MSB, char LSB);

int main(){
  


  
  
  float sum = 0;
  int sum_i = 0;

  dof_class dof;
  
  
  /*flop math speed tests - slow!!
   *
   */
  volatile int t = CNT;
  for (int i = 0; i<100; i++)
  {
    sum += i;
  } 
  
  volatile int t1 = CNT;
  print("\nTime per flop summation: %f, %u, %u sum: %f", (float)(t1-t)/(float)CLKFREQ/100, t1, t, sum);
  
  
  
  sum = 10000;
 
  t = CNT;
  for (int i = 1; i<101; i++)
  {
    sum -= i;
     
  }  
  t1 = CNT;
  
  print("\nTime per flop subtraction: %f, %u, %u, %f", (float)(t1-t)/(float)CLKFREQ/100, t1, t , sum);
  
  
  sum = 1;
 
  t = CNT;
  for (int i = 1; i<11; i++)
  {
    sum *= 3;
     
  }  
  t1 = CNT;
  
  print("\nTime per flop multiplication: %f, %u, %u, %f", (float)(t1-t)/(float)CLKFREQ/10, t1, t , sum);
  
  
  
  
  
  sum = 10000;
  t = CNT;
  for (int i = 1; i<101; i++)
  {
    sum /= 1.023;
     
  }  
  t1 = CNT;
  
  print("\nTime per flop division: %f, %u, %u, %f", (float)(t1-t)/(float)CLKFREQ/100, t1, t , sum);
  
  
  /*Integer math speeds - much faster!
  */
  
  
  sum_i = 1;
  t = CNT;
  for (int i = 1; i<11; i++)
  {
    sum_i *= 13;
     
  }  
  t1 = CNT;
 
  print("\nTime per integer multiplication: %f, %u, %u, %d", (float)(t1-t)/(float)CLKFREQ/10, t1, t , sum_i);
  
  sum_i = 100000;
  t = CNT;
  for (int i = 1; i<11; i++)
  {
    sum_i /= 3;
     
  }  
  t1 = CNT;
  
  print("\nTime per integer division: %f, %u, %u, %d\n\n\n", (float)(t1-t)/(float)CLKFREQ/10, t1, t , sum_i);
  
  
  
  
  
  t = CNT;
  for (int i = 0; i<100; i++)
  {
    dof.update();
  } 
  t1 = CNT;
  print("\nTime per sample: %f", (float)(t1-t)/(float)CLKFREQ/100 );
  print("\nAccelerometer data: %f, %f, %f", dof.Acc_data.x, dof.Acc_data.y, dof.Acc_data.z);
  print("\nGyroscope data: %f, %f, %f",     dof.Gyr_data.x, dof.Gyr_data.y, dof.Gyr_data.z);
  print("\nMagnetometer data: %f, %f, %f",  dof.Mag_data.x, dof.Mag_data.y, dof.Mag_data.z);
  
  while(1);
  return 0;
}

