
   // See Cillian's logbook, semester 2 week 5, for explanation of filters. 
  

//Never use arrays if at all posible, avoid extra overhead

//This whole file should probably be reimplimented in fixeding point math. 




#include <stdio.h>            // Recommended over iostream for saving space
#include <propeller.h>        // Propeller-specific functions
#include "simpletools.h"

#include "filters.h"
#include "fixed.h"

fixed accel_angle_p = 0;
fixed accel_angle_p_old = 0;

fixed filt_angle_p = 0;
fixed filt_angle_p_old = 0;

fixed gyro_p = 0;
fixed gyro_p_old = 0;


fixed accel_angle_r = 0;
fixed accel_angle_r_old = 0;

fixed filt_angle_r = 0;
fixed filt_angle_r_old = 0;

fixed gyro_r = 0;
fixed gyro_r_old = 0;


//filter values - THESE ARE THE ONLY ONES THAT SHOULD CHANGE

extern  float _sample_period;

 fixed tau(0.4);   //NB. Specific pitch values
 
fixed Ts(_sample_period);


 fixed alpha = Ts - (tau*2);  //precompute to avoid floating point operations
 float _beta  = 1/(Ts.f() + tau.f()*2);  //inverse of beta in cillians logbook
 fixed beta(_beta);
 fixed rho((tau * Ts).f());


fixed pitch_comp_filter(fixed accel_angle, fixed gyro)
{
      //print("sample_period1: %f, sample_period2: %f", Ts.f(), (Ts - (tau*2)).f());
      //print("\n%f, %f, %f, %f, %f", tau.f(), Ts.f(), alpha.f(), beta.f(), rho.f());
      //while(1);
	//this should probably be done in fixed point math - drop to 2/3 fixeding point ops per loop

	//advance sample point by 1
	accel_angle_p_old = accel_angle_p;
	accel_angle_p = accel_angle;

	gyro_p_old = gyro_p;
	gyro_p = gyro;

	filt_angle_p_old = filt_angle_p;

	filt_angle_p = beta * (    Ts * (accel_angle_p + accel_angle_p_old)  - 
		                     alpha * filt_angle_p_old  + 
		                     rho * (gyro_p_old + gyro_p)   );


	return filt_angle_p;
}



fixed roll_comp_filter(fixed accel_angle, fixed gyro)
{
	//this should probably be done in fixed point math - drop to 3 fixeding point ops per loop

	//advance sample point by 1
	accel_angle_r_old = accel_angle_r;
	accel_angle_r = accel_angle;

	gyro_r_old = gyro_r;
	gyro_r = gyro;

	filt_angle_r_old = filt_angle_r;

	filt_angle_r = beta * (  Ts * (accel_angle_r + accel_angle_r_old)  - 
		                  alpha * filt_angle_r_old  + 
		                  rho * (gyro_r_old + gyro_r)   );

	return filt_angle_r;
}