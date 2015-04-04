/* See Ross' Logbook, semester 2 week 3 for control difference equations. 
 * Gains are DILIBERATELY ENTERED AS FLOATS, AND NOT A FLOAT ARRAY - faster, no referencing overhead. 

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

//controller gains (1-9 for 4 controllers)
const float kh1 =  1.00000000; const float kh2 =  1.94143434; const float kh3 = -2.05856566; const float kh4 = 2.45649993; 
const float kh5 = -2.33936861; const float kh6 =  0.65588425; const float kh7 = -0.65588425; const float kh8 = 0.59261116; const float kh9 = -0.59261116;

const float kr1 =  1.00000000; const float kr2 =  0.06995984; const float kr3 = -0.18004016; const float kr4 = 1.67745556; 
const float kr5 = -1.56737523; const float kr6 =  0.32608685; const float kr7 = -0.32608685; const float kr8 = 0.83837179; const float kr9 = -0.83837179;

const float kp1 =  1.00000000; const float kp2 =  0.06995984; const float kp3 = -0.18004016; const float kp4 = 1.67745556; 
const float kp5 = -1.56737523; const float kp6 =  0.32608685; const float kp7 = -0.32608685; const float kp8 = 0.83837179; const float kp9 = -0.838371579;

const float ky1 =  1.00000000; const float ky2 = -0.00394348; const float ky3 = -0.00394348; const float ky4 = 0.37474411; 
const float ky5 = -0.36685716; const float ky6 =  0.45170301; const float ky7 = -0.45170301; const float ky8 = 0.11704666; const float ky9 = -0.11704666;





//The height observer gains are:  
//          Fn1                             Fn2                            Fn3                            Gn1
const float Fh11 = -1.06012243; const float Fh12 = 0.03000000; const float Fh13 = 0.00658443; const float Gh14 =  2.06012243;
const float Fh21 = -38.4669769; const float Fh22 = 1.00000000; const float Fh23 = 0.40978132; const float Gh24 = 38.46697692;
const float Fh31 = -9.37529529; const float Fh32 = 0.00000000; const float Fh33 = 0.65143906; const float Gh34 =  9.37529529;

//The roll observer gains are:  
//          Fn1                            Fn2                             Fn3                            Gn1                             Gn2 
const float Fr11 = 0.50291653; const float Fr12 = -0.00243995; const float Fr13 = 0.02059603; const float Gr14 =  0.49708347; const float Gr15 = 0.03243995;
const float Fr21 = 0.12765187; const float Fr22 =  0.34837765; const float Fr23 = 1.28179111; const float Gr24 = -0.12765187; const float Gr25 = 0.65162235;
const float Fr31 = 0.01464762; const float Fr32 = -0.01783035; const float Fr33 = 0.65143906; const float Gr34 = -0.01464762; const float Gr35 = 0.01783035;

//The pitch observer gains are:  
//          Fn1                            Fn2                             Fn3                            Gn1                             Gn2 
const float Fp11 = 0.50291653; const float Fp12 = -0.00243995; const float Fp13 = 0.02059603; const float Gp14 =  0.49708347; const float Gp15 = 0.03243995;
const float Fp21 = 0.12765187; const float Fp22 =  0.34837765; const float Fp23 = 1.28179111; const float Gp24 = -0.12765187; const float Gp25 = 0.65162235;
const float Fp31 = 0.01464762; const float Fp32 = -0.01783035; const float Fp33 = 0.65143906; const float Gp34 = -0.01464762; const float Gp35 = 0.01783035;

//The yaw observer gains are:  
//          Fn1                            Fn2                             Fn3                            Gn1                             Gn2 
const float Fy11 = 0.50291653; const float Fy12 = -0.00243995; const float Fy13 = 0.02059603; const float Gy14 =  0.19050553; const float Gy15 = 0.02745124;
const float Fy21 = 0.12765187; const float Fy22 =  0.34837765; const float Fy23 = 1.28179111; const float Gy24 = -0.00549183; const float Gy25 = 0.03271282;
const float Fy31 = 0.01464762; const float Fy32 = -0.01783035; const float Fy33 = 0.65143906; const float Gy34 =  0.00913020; const float Gy35 = 0.25971804;



float x1hat_h_k 0; float x1hat_h_k_minus_1 0;
float x2hat_h_k 0; float x2hat_h_k_minus_1 0;
float x3hat_h_k 0; float x3hat_h_k_minus_1 0;

float x1hat_r_k 0; float x1hat_r_k_minus_1 0;
float x2hat_r_k 0; float x2hat_r_k_minus_1 0;
float x3hat_r_k 0; float x3hat_r_k_minus_1 0;

float x1hat_p_k 0; float x1hat_p_k_minus_1 0;
float x2hat_p_k 0; float x2hat_p_k_minus_1 0;
float x3hat_p_k 0; float x3hat_p_k_minus_1 0;

float x1hat_y_k 0; float x1hat_y_k_minus_1 0;
float x2hat_y_k 0; float x2hat_y_k_minus_1 0;
float x3hat_y_k 0; float x3hat_y_k_minus_1 0;

float setPoint_h_k = 0; float setPoint_h_k_minus_1 = 0;
float setPoint_r_k = 0; float setPoint_r_k_minus_1 = 0;
float setPoint_p_k = 0; float setPoint_p_k_minus_1 = 0;
float setPoint_y_k = 0; float setPoint_y_k_minus_1 = 0;


float u_h_k = 0; float u_h_k_minus_1 = 0;
float u_r_k = 0; float u_r_k_minus_1 = 0;
float u_p_k = 0; float u_p_k_minus_1 = 0;
float u_y_k = 0; float u_y_k_minus_1 = 0;




float control_pitch(float theta, float gyr_x)
{
	
	_pitch_observer(float theta, float gyr_x);

	u_h_k = k21*u_h_k_minus_1+ k22*setPoint_h_k + k23*setPoint_h_k_minus_1 + k24*x1hat_h_k + k15*x1hat_h_k_minus_1
	      + k16*x2hat_h_k + k17*x2hat_h_k_minus_1 + k18*x3hat_h_k + k19*x3hat_h_k_minus_1;
}

float _pitch_observer(float theta, float gyr_x)  //this should not be called other than inside control_pitch()
{
	x1hat_h_k = Fh11*x1hat_h_k_minus_1+ Fh12*x2hat_h_k_minus_1 + Fh13*x3hat_h_k_minus_1 + Gh11*theta + Gh12*gyr_x;
 	x2hat_h_k = Fh21*x1hat_h_k_minus_1+ Fh22*x2hat_h_k_minus_1 + Fh23*x3hat_h_k_minus_1 + Gh21*theta + Gh22*gyr_x;
 	x3hat_h_k = Fh31*x1hat_h_k_minus_1+ Fh32*x2hat_h_k_minus_1 + Fh33*x3hat_h_k_minus_1 + Gh31*theta + Gh32*gyr_x;
}




float control_roll(float phi, float gyr_y)
{
	
	_roll_observer(float phi, float gyr_y);

	u_r_k = k11*u_r_k_minus_1+ k12*setPoint_r_k + k13*setPoint_r_k_minus_1 + k14*x1hat_r_k + k15*x1hat_r_k_minus_1
	      + k16*x2hat_r_k + k17*x2hat_r_k_minus_1 + k18*x3hat_r_k + k19*x3hat_r_k_minus_1;
}

float _roll_observer(float phi, float gyr_y)  //this should not be called other than inside control_pitch()
{
	x1hat_r_k = Fh11*x1hat_r_k_minus_1+ Fh12*x2hat_r_k_minus_1 + Fh13*x3hat_r_k_minus_1 + Gh11*phi + Gh12*gyr_y;
 	x2hat_r_k = Fh21*x1hat_r_k_minus_1+ Fh22*x2hat_r_k_minus_1 + Fh23*x3hat_r_k_minus_1 + Gh21*phi + Gh22*gyr_y;
 	x3hat_r_k = Fh31*x1hat_r_k_minus_1+ Fh32*x2hat_r_k_minus_1 + Fh33*x3hat_r_k_minus_1 + Gh31*phi + Gh32*gyr_y;
}




u(k)= K1*u(k-1)+ K2*r(k) + K3*r(k-1) + K4*x1hat(k) + K5*x1hat(k-1) + K6*x2hat(k) + K7*x2hat(k-1) + K8*x3hat(k) + K9*x3hat(k-1)



x1hat(k+1)= F11*x1hat(k)+ F12*x2hat(k) + F13*x3hat(k) + G11*y1(k) + G12*y2(k) 
x2hat(k+1)= F21*x1hat(k)+ F22*x2hat(k) + F23*x3hat(k) + G21*y1(k) + G22*y2(k) 
x3hat(k+1)= F31*x1hat(k)+ F32*x2hat(k) + F33*x3hat(k) + G31*y1(k) + G32*y2(k) 