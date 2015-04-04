/**
* This is the main Blank Simple C++ Project program file.
*/

#include "simpletools.h"
#include "pwm32.h"

using namespace libpropeller;

PWM32 pwm1; //PWM32 constructor

int main(){
  // Add your code here

  int in;
  
  pwm1.Start();
  
  /*wire pin 17 to ground through a 10k resistor.
   *connect 16 directly to 17 */
  volatile int t;
  int dt = CLKFREQ*0.02;
  pwm1.PWM(20, 900, 19100);
  pause(2000);
  pwm1.PWM(20, 1200, 18800);
  pause(1000);
  
  while(1)
  {
      t = CNT;
      in = input(26);
      if (in==1)
      {
          pwm1.PWM(20, 1500, 18500);
      }          
      else
      {
          pwm1.PWM(20, 1700, 18300);
      } 
      while( CNT-t<dt);    
  }
  
  
  pwm1.PWM(17, 1500, 15000-1500);
  /*
  print("\npulse time: %d us", duration1);
  print("\npulse time: %d us", duration2);
  print("\npulse time: %d us", duration3);
    */
    
  
  return 0;
}
