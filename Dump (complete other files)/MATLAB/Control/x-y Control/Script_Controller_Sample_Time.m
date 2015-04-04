%% Print all Controller Gains

fileID = fopen('Controll_Difference_Equations.dat','w');
fprintf(fileID,' //Channel 1, u1 is Height\n// Channel 2, u2 is Roll\n// Channel 3, u3 is Pitch\n// Channel 4, u4 is Yaw\n');
fprintf(fileID,'\n//Motor 1 PWM input is summed +u1-u3+u4\n// Motor 2 PWM input summed +u1-u2-u4\n// Motor 3 PWM input is summed +u1+u3+u4\n// Motor 4 PWM input summed +u1+u2-u4 \n');
fprintf(fileID,'\n//Height Channel feeds are summed ++++\n// Roll Channel feeds are summed 0-0+\n// Pitch Channel Feeds are summed -0+0\n// Yaw channel feeds are summed +-+- \n');

fprintf(fileID,'\n//The regulator controller with forward-Eulers general formula is:  \n');
fprintf(fileID,'// u(k)= K1*u(k-1)+ K2*r(k-1) + K3*r(k-1) + K4*x1hat(k-1) + K5*x1hat(k-1) + K6*x2hat(k-1) + K7*x3hat(k-1)\n');

fprintf(fileID,'\n//  The regulator controller with backwards-Eulers general formula is:  \n');
fprintf(fileID,'// u(k)= K1*r(k) + K2*v(k) + K3*x1hat(k) + K4*x2hat(k) + K5*x3hat(k)');
fprintf(fileID,'\n const float kh1 = %12.8f;     const float kh2 = %12.8f;     const float kh3 = %12.8f;     const float kh4 = %12.8f;   const float kh5 = %12.8f;',     Feed_Forward_Height_d,  1,-K_Height_d(1),   -K_Height_d(2), -K_Height_d(3));
fprintf(fileID,'\n const float kr1 = %12.8f;     const float kr2 = %12.8f;     const float kr3 = %12.8f;     const float kr4 = %12.8f;   const float kr5 = %12.8f;',     Feed_Forward_Roll_d,    1,-K_Roll_d(1),     -K_Roll_d(2),   -K_Roll_d(3));
fprintf(fileID,'\n const float kp1 = %12.8f;     const float kp2 = %12.8f;     const float kp3 = %12.8f;     const float kp4 = %12.8f;   const float kp5 = %12.8f;',     Feed_Forward_Roll_d,    1,-K_Roll_d(1),     -K_Roll_d(2),   -K_Roll_d(3));
fprintf(fileID,'\n const float ky1 = %12.8f;     const float ky2 = %12.8f;     const float ky3 = %12.8f;     const float ky4 = %12.8f;   const float ky5 = %12.8f;\n',   Feed_Forward_Yaw_d,     1,-K_Yaw_d(1),      -K_Yaw_d(2),    -K_Yaw_d(3));

fprintf(fileID,'\n// The output of the backwards-Eulers integrator is:  \n');
fprintf(fileID,'// v(k)= v(k-1) + KiTs*[r(k) - x1hat(k)]\n');
fprintf(fileID,'// This the where the clamping on the output should be applied\n');
fprintf(fileID,'\n const float khi = %12.8f;',-K_Height_i*Ts);
fprintf(fileID,'\n const float kri = %12.8f;',-K_Roll_i*Ts);
fprintf(fileID,'\n const float kpi = %12.8f;',-K_Roll_i*Ts);
fprintf(fileID,'\n const float kyi = %12.8f;',-K_Yaw_i*Ts);


fprintf(fileID,'\n\n//The regulator controller with Tustisn general formula is:  \n');
fprintf(fileID,'// u(k)= K1*u(k-1)+ K2*r(k) + K3*r(k-1) + K4*x1hat(k) + K5*x1hat(k-1) + K6*x2hat(k) + K7*x2hat(k-1) + K8*x3hat(k) + K9*x3hat(k-1)');
fprintf(fileID,'\n\n// Where\n //Ki is the ith element of the controller matrix for a particular channel (Height, Roll, Pitch, Yaw)\n// r is the setpoint signal \n// u is the channel signal \n// x1hat is the position or angle estimate for that system \n// x2hat is the rate of change of x1 estimate \n// x3hat is the motor state estimate for a channel \n');

fprintf(fileID,'\n//The controller gains for above controller with tustins method and no clamping is:  \n');
t=transpose(U_Gains);
fprintf(fileID,'const float kh1 = %12.8f;  const float kh2 = %12.8f; const float kh3 =  %12.8f; const float kh4 =  %12.8f; const float kh5 =  %12.8f; const float kh6 =  %12.8f; const float kh7 =  %12.8f; const float kh8 =  %12.8f; const float kh9 =  %12.8f;\n',t(1:9));
fprintf(fileID,'const float kr1 = %12.8f;  const float kr2 = %12.8f; const float kr3 =  %12.8f; const float kr4 =  %12.8f; const float kr5 =  %12.8f; const float kr6 =  %12.8f; const float kr7 =  %12.8f; const float kr8 =  %12.8f; const float kr9 =  %12.8f;\n',t(10:18));
fprintf(fileID,'const float kp1 = %12.8f;  const float kp2 = %12.8f; const float kp3 =  %12.8f; const float kp4 =  %12.8f; const float kp5 =  %12.8f; const float kp6 =  %12.8f; const float kp7 =  %12.8f; const float kp8 =  %12.8f; const float kp9 =  %12.8f;\n',t(19:27));
fprintf(fileID,'const float ky1 = %12.8f;  const float ky2 = %12.8f; const float ky3 =  %12.8f; const float ky4 =  %12.8f; const float ky5 =  %12.8f; const float ky6 =  %12.8f; const float ky7 =  %12.8f; const float ky8 =  %12.8f; const float ky9 =  %12.8f;\n',t(28:36));
% fprintf(fileID,'//%12s %12s %12s %12s %12s %12s %12s %12s %12s\n','k1','k2','k3','k4','k5','k6','k7','k8','k9');
% fprintf(fileID,'//%12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f\n',transpose(U_Gains));
% fprintf(fileID,'\n\n//Where 1st row is for channel 1, 2nd for channel 2, etc\n');

fprintf(fileID,'\n\n// The states for each controller must be observed, this means there should be 4 sets of the 3 observer equations below\n');
fprintf(fileID,'\n// The observer general formula is:  \n');
fprintf(fileID,' // x1hat(k+1)= F11*x1hat(k)+ F12*x2hat(k) + F13*x3hat(k) + G11*y1(k) + G12*y2(k) \n// x2hat(k+1)= F21*x1hat(k)+ F22*x2hat(k) + F23*x3hat(k) + G21*y1(k) + G22*y2(k) \n// x3hat(k+1)= F31*x1hat(k)+ F32*x2hat(k) + F33*x3hat(k) + G31*y1(k) + G32*y2(k) ');
fprintf(fileID,'\n\n// Where\n// x1hat is estimated x1 value for position/angle\n// x2hat is estimated x2 value for postion/angle rate\n// x3hat is estimated x3 value is the motor state\n// y1 is output of position/angle sensor \n// y2 is output of rate sensor\n');


fprintf(fileID,'\n\n// The height observer gains are:  \n');
t=transpose(Height_Obs);
fprintf(fileID,'const float Fh11 = %12.8f;  const float Fh12 = %12.8f; const float Fh13 =  %12.8f  const float Gh11 =  %12.8f; \n',t(1:4));
fprintf(fileID,'const float Fh21 = %12.8f;  const float Fh22 = %12.8f; const float Fh23 =  %12.8f  const float Gh21 =  %12.8f; \n',t(5:8));
fprintf(fileID,'const float Fh31 = %12.8f;  const float Fh32 = %12.8f; const float Fh33 =  %12.8f  const float Gh31 =  %12.8f; \n',t(9:12));


fprintf(fileID,'\n//The roll observer gains are:  \n');
t=transpose(Roll_Obs);
fprintf(fileID,'const float Fr11 = %12.8f;  const float Fr12 = %12.8f; const float Fr13 =  %12.8f  const float Gr11 =  %12.8f;  const float Gr12 =  %12.8f; \n',t(1:5));
fprintf(fileID,'const float Fr21 = %12.8f;  const float Fr22 = %12.8f; const float Fr23 =  %12.8f  const float Gr21 =  %12.8f;  const float Gr22 =  %12.8f; \n',t(6:10));
fprintf(fileID,'const float Fr31 = %12.8f;  const float Fr32 = %12.8f; const float Fr33 =  %12.8f  const float Gr31 =  %12.8f;  const float Gr32 =  %12.8f; \n',t(11:15));
% fprintf(fileID,'// %12s %12s %12s %12s %12s \n','Fn1','Fn2','Fn3','Gn1','Gn2');
% fprintf(fileID,'// %12.8f %12.8f %12.8f %12.8f %12.8f\n',transpose(Roll_Obs));

fprintf(fileID,'\n// The pitch observer gains are:  \n');
t=transpose(Pitch_Obs);
fprintf(fileID,'const float Fp11 = %12.8f;  const float Fp12 = %12.8f; const float Fp13 =  %12.8f  const float Gp11 =  %12.8f;  const float Gp12 =  %12.8f; \n',t(1:5));
fprintf(fileID,'const float Fp21 = %12.8f;  const float Fp22 = %12.8f; const float Fp23 =  %12.8f  const float Gp21 =  %12.8f;  const float Gp22 =  %12.8f; \n',t(6:10));
fprintf(fileID,'const float Fp31 = %12.8f;  const float Fp32 = %12.8f; const float Fp33 =  %12.8f  const float Gp31 =  %12.8f;  const float Gp32 =  %12.8f; \n',t(11:15));
% fprintf(fileID,'// %12s %12s %12s %12s %12s \n','Fn1','Fn2','Fn3','Gn1','Gn2');
% fprintf(fileID,'// %12.8f %12.8f %12.8f %12.8f %12.8f\n',transpose(Pitch_Obs));

fprintf(fileID,'\n// The yaw observer gains are:  \n');
t=transpose(Yaw_Obs);
fprintf(fileID,'const float Fy11 = %12.8f;  const float Fy12 = %12.8f; const float Fy13 =  %12.8f  const float Gy11 =  %12.8f;  const float Gy12 =  %12.8f; \n',t(1:5));
fprintf(fileID,'const float Fy21 = %12.8f;  const float Fy22 = %12.8f; const float Fy23 =  %12.8f  const float Gy21 =  %12.8f;  const float Gy22 =  %12.8f; \n',t(6:10));
fprintf(fileID,'const float Fy31 = %12.8f;  const float Fy32 = %12.8f; const float Fy33 =  %12.8f  const float Gy31 =  %12.8f;  const float Gy32 =  %12.8f; \n',t(11:15));
% fprintf(fileID,'// %12s %12s %12s %12s %12s \n','Fn1','Fn2','Fn3','Gn1','Gn2');
% fprintf(fileID,'// %12.8f %12.8f %12.8f %12.8f %12.8f\n',transpose(Yaw_Obs));

fprintf(fileID,'\n// I believe the outputs will pass from sensor to Kalman then through observer. \n// The observed states will then be used in controller. \n// Controller Outputs will then be summed together\n');

fclose(fileID);
type Controll_Difference_Equations.dat