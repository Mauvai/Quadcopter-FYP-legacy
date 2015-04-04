/* See Ross' Logbook, semester 2 week 3 for control difference equations. 
 * Gains are DILIBERATELY ENTERED AS fixedS, AND NOT A fixed ARRAY - faster, no referencing overhead. 

 *Channel 1, u1 is Height
 *Channel 2, u2 is Roll
 *Channel 3, u3 is Pitch
 *Channel 4, u4 is Yaw

 *Motor 1 PWM input is summed +u1   -u3+u4
 *Motor 2 PWM input summed    +u1-u2   -u4
 *Motor 3 PWM input is summed +u1   +u3+u4
 *Motor 4 PWM input summed    +u1+u2   -u4 
 */
 

#include <stdio.h>            // Recommended over iostream for saving space
#include <propeller.h>        // Propeller-specific functions
#include "simpletools.h"
#include "control.h"
#include "fixed.h"

//declare observers here to keep them "private" 
void _height_observer(fixed height            );  
void  _pitch_observer(fixed theta, fixed gyr_x, fixed pwm);
void   _roll_observer(fixed phi,   fixed gyr_y, fixed pwm);
void    _yaw_observer(fixed phi,   fixed gyr_z);

//integrator clamp function, keeps values between 1.1 and 1.95 ms
inline fixed _clamp(fixed number);




//controller gains
/*
 fixed kh1 =   2.00000000;      fixed kh2 =   1.00000000;      fixed kh3 =  -2.39793427;      fixed kh4 =  -0.65588425;    fixed kh5 =  -0.59261116;
 fixed kr1 =   0.12500000;      fixed kr2 =   1.00000000;      fixed kr3 =  -1.62241539;      fixed kr4 =  -0.32608685;    fixed kr5 =  -0.83837179;
 fixed kp1 =   0.12500000;      fixed kp2 =   1.00000000;      fixed kp3 =  -1.62241539;      fixed kp4 =  -0.32608685;    fixed kp5 =  -0.83837179;
 fixed ky1 =   0.00000000;      fixed ky2 =   1.00000000;      fixed ky3 =  -0.37080064;      fixed ky4 =  -0.45170301;    fixed ky5 =  -0.11704666;
*/
fixed kh1 =   2.00000000;     fixed kh2 =   1.00000000;     fixed kh3 =  -2.48783180;     fixed kh4 =  -0.67321731;   fixed kh5 =  -0.60592245;
 fixed kr1 =   0.12500000;     fixed kr2 =   1.00000000;     fixed kr3 =  -1.70465281;     fixed kr4 =  -0.33244746;   fixed kr5 =  -0.85195044;
 fixed kp1 =   0.12500000;     fixed kp2 =   1.00000000;     fixed kp3 =  -1.70465281;     fixed kp4 =  -0.33244746;   fixed kp5 =  -0.85195044;
 fixed ky1 =   0.00000000;     fixed ky2 =   1.00000000;     fixed ky3 =  -0.37262724;     fixed ky4 =  -0.45367895;   fixed ky5 =  -0.11754213;

/*
//The height observer gains are:  
//    Fn1                         Fn2                       Fn3                      Gn1
fixed Fh11 =  -1.06012243;  fixed Fh12 =  0.03000000; fixed Fh13 = 0.00658443; fixed Gh11 =   2.06012243; 
fixed Fh21 = -38.46697692;  fixed Fh22 =  1.00000000; fixed Fh23 = 0.40978132; fixed Gh21 =  38.46697692; 
fixed Fh31 =  -9.37529529;  fixed Fh32 =  0.00000000; fixed Fh33 = 0.65143906; fixed Gh31 =   9.37529529; 

//The roll observer gains are:  
fixed Fr11 =   0.26612905;  fixed Fr12 = -0.00622157; fixed Fr13 = 0.02059603;  fixed Gr11 =  0.73387095;  fixed Gr12 = 0.03622157; fixed Br1 = 0.00304427; 
fixed Fr21 =   0.07126057;  fixed Fr22 = -0.12257106; fixed Fr23 = 1.28179111;  fixed Gr21 = -0.07126057;  fixed Gr22 = 1.12257106; fixed Br2 = 0.29422898; 
fixed Fr31 =   0.02128054;  fixed Fr32 = -0.11676537; fixed Fr33 = 0.65143906;  fixed Gr31 = -0.02128054;  fixed Gr32 = 0.11676537; fixed Br3 = 0.34856094; 

// The pitch observer gains are:  
fixed Fp11 =   0.26612905;  fixed Fp12 = -0.00622157; fixed Fp13 = 0.02059603;  fixed Gp11 =  0.73387095;  fixed Gp12 = 0.03622157; fixed Bp1 = 0.00304427; 
fixed Fp21 =   0.07126057;  fixed Fp22 = -0.12257106; fixed Fp23 = 1.28179111;  fixed Gp21 = -0.07126057;  fixed Gp22 = 1.12257106; fixed Bp2 = 0.29422898;
fixed Fp31 =   0.02128054;  fixed Fp32 = -0.11676537; fixed Fp33 = 0.65143906;  fixed Gp31 = -0.02128054;  fixed Gp32 = 0.11676537; fixed Bp3 = 0.34856094; 

// The yaw observer gains are:  
fixed Fy11 =   0.26612905;  fixed Fy12 = -0.00622157; fixed Fy13 = 0.02059603;  fixed Gy11 =  0.19050553;  fixed Gy12 = 0.02745124; fixed By1 = 0.00023151; 
fixed Fy21 =   0.07126057;  fixed Fy22 = -0.12257106; fixed Fy23 = 1.28179111;  fixed Gy21 = -0.00549183;  fixed Gy22 = 0.03271282; fixed By2 = 0.02224443;
fixed Fy31 =   0.02128054;  fixed Fy32 = -0.11676537; fixed Fy33 = 0.65143906;  fixed Gy31 =  0.00913020;  fixed Gy32 = 0.25971804; fixed By3 = 0.34856094;
*/


fixed Fh11 =  -0.74010653;  fixed Fh12 =   0.02000000; fixed Fh13 =    0.00306067;  fixed Gh11 =    1.74010653;  fixed Bh1 =    0.00029830;  
fixed Fh21 = -42.14233391;  fixed Fh22 =   1.00000000; fixed Fh23 =    0.29217262;  fixed Gh21 =   42.14233391;  fixed Bh2 =    0.04372382; 
fixed Fh31 = -13.98804501;  fixed Fh32 =   0.00000000; fixed Fh33 =    0.75147729;  fixed Gh31 =   13.98804501;  fixed Bh3 =    0.24852271; 

// The roll observer gains are:  
fixed Fr11 =   0.44058870;  fixed Fr12 =  -0.00325957; fixed Fr13 =    0.00957373;  fixed Gr11 =    0.55941130;  fixed Gr12 =    0.02325957; fixed Br1 =    0.00093307; 
fixed Fr21 =   0.04789254;  fixed Fr22 =   0.12823289; fixed Fr23 =    0.91391248;  fixed Gr21 =   -0.04789254;  fixed Gr22 =    0.87176711; fixed Br2 =    0.13676758; 
fixed Fr31 =   0.01612799;  fixed Fr32 =  -0.10614460; fixed Fr33 =    0.75147729;  fixed Gr31 =   -0.01612799;  fixed Gr32 =    0.10614460; fixed Br3 =    0.24852271; 

// The pitch observer gains are:  
fixed Fp11 =   0.44058870;  fixed Fp12 =  -0.00325957; fixed Fp13 =    0.00957373;  fixed Gp11 =    0.55941130;  fixed Gp12 =    0.02325957;  fixed Bp1 =    0.00093307; 
fixed Fp21 =   0.04789254;  fixed Fp22 =   0.12823289; fixed Fp23 =    0.91391248;  fixed Gp21 =   -0.04789254;  fixed Gp22 =    0.87176711;  fixed Bp2 =    0.13676758; 
fixed Fp31 =   0.01612799;  fixed Fp32 =  -0.10614460; fixed Fp33 =    0.75147729;  fixed Gp31 =   -0.01612799;  fixed Gp32 =    0.10614460;  fixed Bp3 =    0.24852271; 

// The yaw observer gains are:  
fixed Fy11 =   0.44058870;  fixed Fy12 =  -0.00325957; fixed Fy13 =    0.00957373;  fixed Gy11 =    0.12797052;  fixed Gy12 =    0.01872040; fixed By1 =    0.00007156; 
fixed Fy21 =   0.00007156;  fixed Fy22 =   0.04789254; fixed Fy23 =    0.12823289;  fixed Gy21 =    0.91391248;  fixed Gy22 =   -0.00368433; fixed By2 =    0.00753193;
fixed Fy31 =   0.01039855;  fixed Fy32 =   0.01039855; fixed Fy33 =    0.01612799;  fixed Gy31 =   -0.10614460;  fixed Gy32 =    0.75147729; fixed By3 =    0.00657218;

//integrator ants
/*
 fixed khi =    0.059;
 fixed kri =    0.055;
 fixed kpi =    0.055;
 fixed kyi =    0.004;
*/

fixed khi =   0.04085061;
 fixed kri =   0.05555128;
 fixed kpi =   0.05555128;
 fixed kyi =   0.00265140;

fixed x1hat_h_k = 0; fixed x1hat_h_k_minus_1 = 0;
fixed x2hat_h_k = 0; fixed x2hat_h_k_minus_1 = 0;
fixed x3hat_h_k = 0; fixed x3hat_h_k_minus_1 = 0;

fixed x1hat_r_k = 0; fixed x1hat_r_k_minus_1 = 0;
fixed x2hat_r_k = 0; fixed x2hat_r_k_minus_1 = 0;
fixed x3hat_r_k = 0; fixed x3hat_r_k_minus_1 = 0;

fixed x1hat_p_k = 0; fixed x1hat_p_k_minus_1 = 0;
fixed x2hat_p_k = 0; fixed x2hat_p_k_minus_1 = 0;
fixed x3hat_p_k = 0; fixed x3hat_p_k_minus_1 = 0;

fixed x1hat_y_k = 0; fixed x1hat_y_k_minus_1 = 0;
fixed x2hat_y_k = 0; fixed x2hat_y_k_minus_1 = 0;
fixed x3hat_y_k = 0; fixed x3hat_y_k_minus_1 = 0;

fixed setPoint_h = 0; 
fixed setPoint_r = 0; 
fixed setPoint_p = 0; 
fixed setPoint_y = 0;

//integrators
fixed integrator_h;
fixed integrator_r;
fixed integrator_p;
fixed integrator_y;

  fixed g1 = 1.186270334163395;   fixed g2 = 0.282566603155235;   fixed g3 = 0.752364056143805;
//fixed g1 = 1.428040025434765;   fixed g2 = 0.315585313898618;   fixed g3 = 0.819404673072734;
//fixed g1 = 0.861020328546044;   fixed g2 = 0.234056627038035;   fixed g3 = 0.648624174726570;
//fixed g1 = 0.661896597580970;   fixed g2 = 0.709032607509754;   fixed g3 = 1.564414323514949;

fixed control_height(fixed height, fixed set_point)
{
	setPoint_h = set_point;

	_height_observer(height);

    integrator_h = _clamp(integrator_h + khi*(setPoint_h - x1hat_h_k));
    
    return kh1*setPoint_h + kh2*integrator_h + kh3*x1hat_h_k + kh4*x2hat_h_k + kh5*x3hat_h_k;

}


void _height_observer(fixed height)  //this should not be called other than inside control_height()
{
	x1hat_h_k = Fp11*x1hat_h_k_minus_1+ Fh12*x2hat_h_k_minus_1 + Fh13*x3hat_h_k_minus_1 + Gh11*height;
 	x2hat_h_k = Fp21*x1hat_h_k_minus_1+ Fh22*x2hat_h_k_minus_1 + Fh23*x3hat_h_k_minus_1 + Gh21*height;
 	x3hat_h_k = Fp31*x1hat_h_k_minus_1+ Fh32*x2hat_h_k_minus_1 + Fh33*x3hat_h_k_minus_1 + Gh31*height;
}



fixed control_pitch(fixed theta, fixed gyr_x, fixed set_point, fixed pwm)
{
	setPoint_p = set_point;

	_pitch_observer(theta, gyr_x, pwm);

	integrator_p = _clamp(integrator_p + kpi*(setPoint_p - x1hat_p_k));
    return kp1*setPoint_p + kp2*integrator_p + kp3*x1hat_p_k + kp4*x2hat_p_k + kp5*x3hat_p_k;
     //return setPoint_p - g1*x1hat_p_k - g2*x2hat_p_k - g3*x3hat_p_k;
    
}

void _pitch_observer(fixed theta, fixed gyr_x, fixed pwm)  //this should not be called other than inside control_pitch()
{
	x1hat_p_k = Fp11*x1hat_p_k_minus_1+ Fp12*x2hat_p_k_minus_1 + Fp13*x3hat_p_k_minus_1 + Gp11*theta + Gp12*gyr_x + Bp1*pwm;
 	x2hat_p_k = Fp21*x1hat_p_k_minus_1+ Fp22*x2hat_p_k_minus_1 + Fp23*x3hat_p_k_minus_1 + Gp21*theta + Gp22*gyr_x + Bp2*pwm;
 	x3hat_p_k = Fp31*x1hat_p_k_minus_1+ Fp32*x2hat_p_k_minus_1 + Fp33*x3hat_p_k_minus_1 + Gp31*theta + Gp32*gyr_x + Bp3*pwm;
     
     x1hat_p_k_minus_1 = x1hat_p_k;
     x2hat_p_k_minus_1 = x2hat_p_k;
     x3hat_p_k_minus_1 = x3hat_p_k;
      
  
}
 


fixed control_roll(fixed phi, fixed gyr_y, fixed set_point, fixed pwm)
{
	setPoint_r = set_point;

	_roll_observer(phi, gyr_y, pwm);

	integrator_r = _clamp(integrator_r + kri*(setPoint_r - x1hat_r_k));
      return kr1*setPoint_r + kr2*integrator_r + kr3*x1hat_r_k + kr4*x2hat_r_k + kr5*x3hat_r_k;
      //return setPoint_r - g1*x1hat_r_k - g2*x2hat_r_k - g3*x3hat_r_k;
}
fixed roll_xhat1(void){return x1hat_r_k;}
fixed roll_xhat2(void){return x2hat_r_k;}
fixed roll_xhat3(void){return x3hat_r_k;}

void _roll_observer(fixed phi, fixed gyr_y, fixed pwm)  //this should not be called other than inside control_roll()
{
	x1hat_r_k = Fr11*x1hat_r_k_minus_1+ Fr12*x2hat_r_k_minus_1 + Fr13*x3hat_r_k_minus_1 + Gr11*phi + Gr12*gyr_y + Br1*pwm;
 	x2hat_r_k = Fr21*x1hat_r_k_minus_1+ Fr22*x2hat_r_k_minus_1 + Fr23*x3hat_r_k_minus_1 + Gr21*phi + Gr22*gyr_y + Br2*pwm;
 	x3hat_r_k = Fr31*x1hat_r_k_minus_1+ Fr32*x2hat_r_k_minus_1 + Fr33*x3hat_r_k_minus_1 + Gr31*phi + Gr32*gyr_y + Br3*pwm;
  
     x1hat_r_k_minus_1 = x1hat_r_k;
     x2hat_r_k_minus_1 = x2hat_r_k;
     x3hat_r_k_minus_1 = x3hat_r_k;
}



fixed control_yaw(fixed psi, fixed gyr_z, fixed set_point)
{
	setPoint_y = set_point;

	_yaw_observer(psi, gyr_z);

	integrator_y = _clamp(integrator_y + kyi*(setPoint_y - x1hat_y_k));
    return ky1*setPoint_y + ky2*integrator_y + ky3*x1hat_y_k + ky4*x2hat_y_k + ky5*x3hat_y_k;	      
}

void _yaw_observer(fixed psi, fixed gyr_z)  //this should not be called other than inside control_yaw()
{
    
    //B TERM NOT ADDED
	x1hat_y_k = Fy11*x1hat_y_k_minus_1+ Fy12*x2hat_y_k_minus_1 + Fy13*x3hat_y_k_minus_1 + Gy11*psi + Gy12*gyr_z;
 	x2hat_y_k = Fy21*x1hat_y_k_minus_1+ Fy22*x2hat_y_k_minus_1 + Fy23*x3hat_y_k_minus_1 + Gy21*psi + Gy22*gyr_z;
 	x3hat_y_k = Fy31*x1hat_y_k_minus_1+ Fy32*x2hat_y_k_minus_1 + Fy33*x3hat_y_k_minus_1 + Gy31*psi + Gy32*gyr_z;
}




//integrator clamp
inline fixed _clamp(fixed number)
{
  //return number;  //disable clamp
  return number;
  if (number > 1U)
  {return fixed(1);}
  else if (number < -1U)
  {return fixed(-1);}
  else
  {return number;}
}  