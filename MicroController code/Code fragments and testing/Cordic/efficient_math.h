
#include "fixed.h"

//cordic solver, redians  See cordic_*.cpp
fixed atan2f_cordic_rad(int Y, int X);  //uses the cordic algorithim to solve for atan2 (Y, X)
fixed table_radians(int loopNo);
fixed cosf_cordic(fixed theta);  //currently only does from -pi/2 to +pi/2!!

//cordic solver, degrees. See cordic_*.cpp
fixed atan2f_cordic_deg(int Y, int X);  //uses the cordic algorithim to solve for atan2 (Y, X)
fixed tan_table_degrees(int loopNo);

