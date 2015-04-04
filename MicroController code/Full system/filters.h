
   // See Cillian's logbook, semester 2 week 1 for explanation of complementary filters. 
  //Not that this file SHOULD NOT be amalgamated with any other filter file, 
 //as the variable names are the same, but will have diferent values. The globals 
//in the corresponding .cpp file have values intended only for the scope of THIS file

#include "fixed.h"


fixed pitch_comp_filter(fixed accel_angle, fixed gyro);
fixed  roll_comp_filter(fixed accel_angle, fixed gyro);



fixed pitch_kalman_filter(fixed pwm, fixed theta, fixed gyr_x);
fixed   return_pitch_rate(void);

fixed  roll_kalman_filter(fixed pwm, fixed phi, fixed gyr_y);
fixed    return_roll_rate(void);



