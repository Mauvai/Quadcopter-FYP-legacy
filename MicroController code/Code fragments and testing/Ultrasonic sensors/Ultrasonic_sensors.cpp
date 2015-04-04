/**
* This is the main Blank Simple C++ Project program file.
*/

//#include <stdio.h>            // Recommended over iostream for saving space
//#include <propeller.h>        // Propeller-specific functions
#include "simpletools.h"


long long int ping_hcsr04(int echopin, int triggerpin);
int ping_cm_hcsr04(int echopin, int triggerpin);


int dt;
volatile int t;            //stores timing values: t holds a clock value, dt is compared to it
volatile int t1;
volatile int t2;

int main(){
  // Add your code here
  int dist;
  dt = CLKFREQ*0.5;    // 2Hz internal sampling  (dt = no of clock cyckes in 1/2 seconds)
  
  int count  = 0;
  t1 = CNT;
  
  while(1)
  {
      count++;
    t = CNT;
    dist = ping_cm_hcsr04(25, 26);
    print("\ndistance: %d", dist);
    
    //print("CNT-t: %d, dt: %d\n", CNT-t, dt);
    
    while(CNT-t < dt)
    {
      //print("delayed");
      pause(1);
     
    } 
    
        
       
  }  
  
  
  return 0;
}

long long int ping_hcsr04(int echopin, int triggerpin)
{
  low(triggerpin);
  pulse_out(triggerpin, 10);
  return pulse_in(echopin, 1);
}

int ping_cm_hcsr04(int echopin, int triggerpin)
{
  long long int tEcho = ping_hcsr04(echopin, triggerpin);
  print("\ntEcho: %d", tEcho);
  int cmDist = tEcho / 58;
  return cmDist;
}
