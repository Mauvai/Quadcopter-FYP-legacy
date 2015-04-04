/**
* This is the main Blank Simple C++ Project program file.
*/

#include <simpletools.h>

#include "fixed.h"
#include "efficient_math.h"

fixed pi(3.141592653590);


int main(){
  // Add your code here

  fixed temp  = cosf_cordic(30*pi/180);
  fixed temp1 = cosf_cordic(-36*pi/180);
  fixed temp2 = cosf_cordic(74*pi/180);
  
  print("\n30: %f", (30*pi/180).f());
  print("\ncos(30): %f, cos(-36): %f, cos(74): %f", temp.f(), temp1.f(), temp2.f());
  //print("\ncos(30): %d", temp.i());
  return 0;
}
