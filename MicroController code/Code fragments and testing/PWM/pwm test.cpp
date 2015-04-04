/*
 4 channel pwm testing
*/

#include "simpletools.h"                      // Include simpletools
#include "pwm32.h"

void pwm2();
int blue  = 12;
int yellow  = 13;
int red  = 14;
int black  = 15;

using namespace libpropeller;

int main()                                    // main function
{
  high(0);
  pause(500);
  low(0);
  pause(500);
  high(0);
  
  PWM32 pwm1; //PWM32 constructor
  pwm1.Start();
  //PWM32 pwm2; //PWM32 constructor
  //pwm1.Start();
  
  //Servo(int pin, int pulse_width) (pulse width in us)
  for (int i = blue; i<=black; i++)
  {
    pwm1.Servo(i, 900); //send arming signal
    pause(5000);           //wait for arm
    pwm1.Servo(i, 1100); //spin low
    pause(2000);
    pwm1.Servo(i, 1600); //spin low
    pause(2000);
    pwm1.Servo(i, 0);
    pause(1000);
  }    
  pwm1.Servo(blue, 1100);
  pwm1.Servo(yellow, 1100);
  pwm1.Servo(red, 1100);
  pwm1.Servo(black, 1100);
  
  pause(2000);
  
  pwm1.Servo(blue, 1600);
  pwm1.Servo(yellow, 1600);
  pwm1.Servo(red, 1600);
  pwm1.Servo(black, 1600);
  
  pause(3000);
  
  pwm1.Servo(blue, 0);
  pwm1.Servo(yellow, 0);
  pwm1.Servo(red, 0);
  pwm1.Servo(black, 0);
  
}


/*  
  pwm1.Servo(24, 100);
  pwm1.Servo(25, 4000);
  pwm1.Servo(26, 9000);
  pwm1.Servo(27, 14000);
  
  pause(1000);
  
  int i1 = 100; 
  int i2 = 4000;
  int i3 = 9000;
  int i4 = 14000;
   
  while(1)
  {
    i1 += 10;
    i2 += 10;
    i3 += 10;
    i4 += 10;
    
    if (i1>=19000)
    {i1 = 100;}
    if (i2>=19000)
    {i2 = 100;}
    if (i3>=19000)
    {i3 = 100;}
    if (i4>=19000)
    {i4 = 100;}
    
    
     pwm1.Servo(24, i1);
     pwm1.Servo(25, i2);
     pwm1.Servo(26, i3);
     pwm1.Servo(27, i4);
    pause(1);
    
  }    
}
 
 
 /*
  unsigned int stack[40+40];
   //print("Hello");
   
   high(0);
   pause(500);
   low(0);
   pause(500);
   high(0);
   
   pwm_start(20000);
   pwm_set(24, 0, 10000);
   pwm_set(25, 1, 100);
   int i = 100;
   int j = 10000;
   //cogstart(&pwm2, NULL, stack, sizeof(stack));
   //cogstart(&adder, NULL, stack, sizeof(stack));
  while(1)                                    // Endless loop
  {
   //print("loop start");
   pwm_set(24, 0, i);
   pwm_set(25, 1, j);
   i = i + 10;
   j = j + 10;
   
   if (i == 19000)
   {
     i = 100;
   } 
   if (j == 19000)
   {
     j = 100;
   }       
       
   pause(1);
  }
}


void pwm2()
{
  pause(250);
  pwm_start(20000);
   pwm_set(26, 0, 10000);
   pwm_set(27, 1, 100);
   int i = 100;
   int j = 10000;
  while(1)                                    // Endless loop
  {
   //print("loop start");
   pwm_set(26, 0, i);
   pwm_set(27, 1, j);
   i = i + 10;
   j = j + 10;
   
   if (i == 19000)
   {
     i = 100;
   } 
   if (j == 19000)
   {
     j = 100;
   }
  }         
}  

*/
