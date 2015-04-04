#include "simpletools.h"
#include "efficient_math.h"

//turns out this isnt actually faster.... its 4 times slower


float fast_sqrt( float number )
{
	int i;  //this MUST be a 32 bit int - ints on the prop system are 32 bits
	float x2, y;
	const float threehalfs = 1.5F;
 
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                       // evil floating point bit level hacking
	i  = 0x5f3759df - ( i >> 1 );               
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
//      y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed
 
	return 1/y;
}
