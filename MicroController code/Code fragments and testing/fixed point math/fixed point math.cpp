/**
* This is the main Blank Simple C++ Project program file.
*/

#include <simpletools.h>
#include "fixed.h"


typedef long long int int64; 
typedef int int32; //int 32 is default into size for this system



int main(){
  // Add your code here

  fixed test(1.1f);
  print("\nfixed: %f", test.f());
  fixed test2(-1.1f);
  test2 = test - test2;
  print("\nfixed: %f", test2.f());
  test2 = test + test2;
  print("\nfixed: %f", test2.f());
  test2 = test * test2;
  print("\nfixed: %f", test2.f());
  test2 = test / test2;
  print("\nfixed: %f", test2.f());
  
  print("\n\nfixed: %x", test2 == test2);
  print("\nfixed: %x",   test2 != test);
  print("\nfixed: %x",   test2 >  1);
  print("\nfixed: %x",   test2 <  test);
  print("\nfixed: %x",   test2 <= test);
  print("\nfixed: %x",   test2 >= test);
  
  
  volatile int t = CNT;
  volatile int t1;
  fixed dummy(1.1f);
  for (int i = 0; i<100; i++)
  {
    fixed temp0((float)i);
    dummy+=temp0;
  }    
  t1 = CNT;
  print("\n\nCreation from float time: %f, %f", (float)(t1-t)/(float)CLKFREQ/(float)100, dummy.f());
  
  t = CNT;
  float x = 0;
  {
  for (int i = 0; i<100; i++)
    
    x+=float(i);
  }    
  t1 = CNT;
  print("\nFloat castand add: %f, %f", (float)(t1-t)/(float)CLKFREQ/(float)100, x);
  
  t = CNT;
  for (int i = 0; i<100; i++)
  {
    
    dummy+=1.0;
  }    
  t1 = CNT;
  print("\nFloat addtion:: %f, %f", (float)(t1-t)/(float)CLKFREQ/(float)100, dummy.f());
  
  
  
  
  
  t = CNT;
  fixed temp1(1.1f);
  fixed temp2(2.2f);
  for (int i = 0; i<100; i++)
  {
    temp1+=i;
  }    
  t1 = CNT;
  print("\n\nAddition time: %f, %f", (float)(t1-t)/(float)CLKFREQ/(float)100, temp1.f());
  
  
  t = CNT;
  temp1 = 10000.0f;
  for (int i = 0; i<100; i++)
  {
    temp1-=i;
  }    
  t1 = CNT;
  print("\nSubtraction time: %f, %f", (float)(t1-t)/(float)CLKFREQ/(float)100, temp1.f());
  
  
  t = CNT;
  temp1 = 1.1f;
  for (int i = 1; i<11; i++)
  {
    temp1*=i;
  }    
  t1 = CNT;
  print("\nMultiplication time: %f, %f", (float)(t1-t)/(float)CLKFREQ/(float)10, temp1.f());
  
  
  
  t = CNT;
  temp1 = 10000;
  for (int i = 1; i<11; i++)
  {
    temp1/=i;
  }    
  t1 = CNT;
  print("\nDivision time: %f, %f", (float)(t1-t)/(float)CLKFREQ/(float)10, temp1.f());
  
  return 0;
}


