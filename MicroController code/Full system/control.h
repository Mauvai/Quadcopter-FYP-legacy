#include "fixed.h"

fixed control_height(fixed height, fixed set_point);    //b term ommited
fixed control_pitch(fixed theta, fixed gyr_x, fixed set_point, fixed pwm);
fixed control_roll(fixed phi, fixed gyr_y, fixed set_point, fixed pwm);
fixed control_yaw(fixed psi, fixed gyr_z, fixed set_point);    //b term ommited

fixed roll_xhat1(void);
fixed roll_xhat2(void);
fixed roll_xhat3(void);