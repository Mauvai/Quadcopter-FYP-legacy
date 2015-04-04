#include "simpletools.h"
#include "cordic.h"



float atan2f_cordic(float Y, float X)
{
          //TODO: need to use more terms and increase resolution
  
        //uses a seven iteration cordic algorithm to solve for atan2
       //see http://forums.parallax.com/showthread.php/127241
      //cordic uses binary searches and integer math to avoid floating
     //point operations and trig calculations - 3ms atan2 drops to 300us with cordic
   
   //NOTE THAT THIS FUNCTION RETURNS DEGREES, NOT RADIANS
  //can easily be modifed by converting the values in tan_table to radians (they are currently degrees)
  
	int x = abs(X*1000);   //3 decimal points of precison preserved, cast to int to avoid floating point math
	int y = abs(Y*1000);  //be aware that floats of size less than 1 are fed into this (generally - abs max is 16. avoid int overflows)
	float sumAngle = 0;  //may want to reimplement sumAngle as fixed point math
	int yNew;
	int xNew;

   //main cordic algorithim - see link
	for (int loopNo = 0; loopNo < 7; loopNo++)
	{
		if (y > 0)
		{
			xNew = x + (y >> loopNo);
			yNew = y - (x >> loopNo);
			sumAngle += tan_table(loopNo);
			
		}
		else if (y < 0)
		{
			xNew = x - (y >> loopNo);
			yNew = y + (x >> loopNo);
			sumAngle -= tan_table(loopNo);
			
		}
		else if (y == 0)
		{
			break;
		}
		x = xNew;
		y = yNew;
	}
    
	
	  //quadrant correction - convert to full 360 degree arc 
    //(atan2 cordic only cumputes 0 - 90 degrees)
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
			return 180 - sumAngle;
		}
		else  //X and Y less than 0     //-90 to -180
		{
			return sumAngle - 180; 
		}
	}
	

	

}

//returns atan(1/(2^looopNo)) in degrees
float tan_table(int loopNo)
{
	switch (loopNo)
	{
	case 0:
		return 45;
	case 1:
		return 26.565;
	case 2:
		return 14.036;
	case 3:
		return 7.125;
	case 4:
		return 3.576;
	case 5:
		return 1.790;
	case 6:
		return 0.895;
	case 7:
		return 0.448;
	}
}