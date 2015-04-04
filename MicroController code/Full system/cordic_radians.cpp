#include "simpletools.h"
#include "efficient_math.h"
#include "fixed.h"

extern const fixed pi;

fixed K(0.607201);

fixed atan2f_cordic_rad(int Y, int X)
{
          //TODO: need to use more terms and increase resolution
  
        //uses a seven iteration cordic algorithm to solve for atan2
       //see http://forums.parallax.com/showthread.php/127241
      //cordic uses binary searches and integer math to avoid floating
     //point operations and trig calculations - 3ms atan2 drops to 300us with cordic
   
   //NOTE THAT THIS FUNCTION RETURNS RADIANS, NOT DEGREES
  //can easily be modifed by converting the values in tan_table to degrees (they are currently radians)
  
	int x = abs(X);   //using int section of a fixed point number - the scaling is irrelevant
	int y = abs(Y);  
	fixed sumAngle(0);  
	int yNew;
	int xNew;

   //main cordic algorithim - see link
	for (int loopNo = 0; loopNo < 7; loopNo++)
	{
		if (y > 0)
		{
			xNew = x + (y >> loopNo);
			yNew = y - (x >> loopNo);
			sumAngle += table_radians(loopNo);
			
		}
		else if (y < 0)
		{
			xNew = x - (y >> loopNo);
			yNew = y + (x >> loopNo);
			sumAngle -= table_radians(loopNo);
			
		}
		else if (y == 0)
		{
			break;
		}
		x = xNew;
		y = yNew;
	}
    
	
        //quadrant correction - convert to full 360 degree arc 
       //(atan2 cordic only computes 0 - 90 degrees)
      //note all of these use capital X and Y, the float inputs
	if (X>=0) //right half of unit circle
	{
		if (Y >= 0) //upper quarter (0 to 90)
		{
			return sumAngle;
		}
		else //0 to -90
		{
			return -sumAngle;
		}
	}
	else //X<0, left half circle  
	{
		if (Y >= 0)  //90 to 180
		{
			return pi - sumAngle;
		}
		else  //X and Y less than 0     //-90 to -180
		{
			return sumAngle - pi; 
		}
	}
	

}


 
fixed cosf_cordic(fixed beta)  //currently only does from -pi/2 to +pi/2!! 
                              //needs optimising pretty badly
{
	
	fixed alpha = 0;     // current angle
	int x = fixed(K).i();  //x is cos(_beta)  //need fixed as 1/K is ~1.6, too small for int
                              //return and operate in ints - saves overhead
	int y = 0;   //y is sin(_beta)
	int xNew, yNew;

      //print("tmp: %d", x.i() >> 0);

	for (int loopNo = 0; loopNo < 7; loopNo++)
	{
		if (alpha < beta)
		{
			xNew = x - (y >> loopNo);
			yNew = y + (x >> loopNo);
			alpha += table_radians(loopNo);
			
		}
		else if (alpha > beta)
		{
			xNew = x + (y >> loopNo);
			yNew = y - (x >> loopNo);
			alpha -= table_radians(loopNo);
			
		}
		else if (alpha == beta)
		{
			break;
		}
		x = xNew;
		y = yNew;
      //print("\nX: %f, Y: %f, alpha: %f", (float)x / (1 << 16), (float)y / (1 << 16), alpha.f());
	}

	return (float)x / (1 << 16);  //unaviodable conversion - int overflows
}

//returns atan(1/(2^looopNo)) in degrees
fixed table_radians(int loopNo)
{
	switch (loopNo)
	{
	case 0:
		return fixed(0.7853982);
	case 1:
		return fixed(0.4636467);
	case 2:
		return fixed(0.2449744);
	case 3:
		return fixed(0.1243547);
	case 4:
		return fixed(0.0624130);
	case 5:
		return fixed(0.0312414);
	case 6:
		return fixed(0.0156207);
	case 7:
		return fixed(0.0078191);
	}
}