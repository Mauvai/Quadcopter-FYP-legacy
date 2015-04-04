

/*implements a static kalman filter for pitch and roll readings from 
a sparkfun 9dof sensor stick, using 3 axis acfcelerometer and gyro readings
*/

#include "filters.h"
#include "fixed.h"


 fixed F_kal_11 =  0.9533;  fixed  F_kal_12 = 0.0097;
 fixed F_kal_21 = -0.0001;  fixed  F_kal_22 = 0.7555;

 fixed H_kal_1  = -0.0006; 
 fixed H_kal_2  =  0.0921; 

 fixed K_kal_11 =  0.0467;  fixed  K_kal_12 = 0.0189;
 fixed K_kal_21 =  0.0001;  fixed  K_kal_22 = 0.2445;


fixed theta_est     = 0;
fixed theta_est_old = 0;
fixed gyr_x_est     = 0;
fixed gyr_x_est_old = 0;

fixed phi_est       = 0;
fixed phi_est_old   = 0;
fixed gyr_y_est     = 0;
fixed gyr_y_est_old = 0;



fixed pitch_kalman_filter(fixed pwm_diff, fixed theta, fixed gyr_x)
{
	//for torque estimations, see Ross' logbook,  semsester 1 week 4. Note we are using a 
	//steady state mapping - a pwm to torque transfer function could be uesd to improve accuracy

	fixed torque_diff =pwm_diff*9.4891;
   //torque_diff = 0;

	theta_est = F_kal_11*theta_est_old + F_kal_12*gyr_x_est_old + H_kal_1*torque_diff + 
	            K_kal_11*theta         + K_kal_12*gyr_x;

	gyr_x_est = F_kal_21*theta_est_old + F_kal_22*gyr_x_est_old + H_kal_2*torque_diff + 
	            K_kal_21*theta         + K_kal_22*gyr_x;

	theta_est_old = theta_est;
	gyr_x_est_old = gyr_x_est;

}
fixed return_pitch_rate(void)
{
	return gyr_x_est;
}

fixed roll_kalman_filter(fixed pwm_diff, fixed phi, fixed gyr_y)
{
	fixed torque_diff = pwm_diff*9.4891;
   //torque_diff = 0;

	phi_est = F_kal_11*phi_est_old + F_kal_12*gyr_y_est_old + H_kal_1*torque_diff + 
	          K_kal_11*phi         + K_kal_12*gyr_y;

	phi_est = F_kal_21*phi_est_old + F_kal_22*gyr_y_est_old + H_kal_2*torque_diff + 
	          K_kal_21*phi         + K_kal_22*gyr_y;

	phi_est_old = phi_est;
	gyr_y_est_old = gyr_y_est;
}
fixed return_roll_rate(void)
{
	return gyr_y_est;
}