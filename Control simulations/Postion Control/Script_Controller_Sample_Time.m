%% Controller Parameters for On-Chip Formulas
F_Height_d  = A_Height_d-G_Height_d*C_Height_d;
F_Roll_d    = A_Roll_d-G_Roll_d*C_Roll_d;
F_Pitch_d   = A_Roll_d-G_Roll_d*C_Roll_d;
F_Yaw_d     = A_Yaw_d-G_Yaw_d*C_Yaw_d;
% Height Controller
Height_u = [1 (K_Height_i*Ts+Feed_Forward_Height_d) (K_Height_i*Ts-Feed_Forward_Height_d) (-K_Height_i*Ts+K_Height_d(1)) (-K_Height_i*Ts-K_Height_d(1)) K_Height_d(2) -K_Height_d(2) K_Height_d(3) -K_Height_d(3)];
% Roll Controller
Roll_u = [1 (K_Roll_i*Ts+Feed_Forward_Roll_d) (K_Roll_i*Ts-Feed_Forward_Roll_d) (-K_Roll_i*Ts+K_Roll_d(1)) (-K_Roll_i*Ts-K_Roll_d(1)) K_Roll_d(2) -K_Roll_d(2) K_Roll_d(3) -K_Roll_d(3)];
% Pitch Controller
Pitch_u = [1 (K_Roll_i*Ts+Feed_Forward_Roll_d) (K_Roll_i*Ts-Feed_Forward_Roll_d) (-K_Roll_i*Ts+K_Roll_d(1)) (-K_Roll_i*Ts-K_Roll_d(1)) K_Roll_d(2) -K_Roll_d(2) K_Roll_d(3) -K_Roll_d(3)];
% Yaw Controller
Yaw_u = [1 (K_Yaw_i*Ts+Feed_Forward_Yaw_d) (K_Yaw_i*Ts-Feed_Forward_Yaw_d) (-K_Yaw_i*Ts+K_Yaw_d(1)) (-K_Yaw_i*Ts-K_Yaw_d(1)) K_Yaw_d(2) -K_Yaw_d(2) K_Yaw_d(3) -K_Yaw_d(3)];
%Controller Matrix
U_Gains= [Height_u;
    Roll_u;
    Pitch_u;
    Yaw_u];
% % Names={'Height';
% %      'Roll';
% %      'Pitch';
% %      'Yaw';};
% %  U_Gains=[Names ,U_Gains]

% Height Observer
Height_Obs = [F_Height_d G_Height_d B_Height_d];
% Roll Observer
Roll_Obs = [F_Roll_d G_Roll_d B_Roll_d];
% Pitch Observer
Pitch_Obs = [F_Roll_d G_Roll_d B_Roll_d];
% Yaw Observer
Yaw_Obs = [F_Roll_d G_Yaw_d B_Yaw_d B_Yaw_d];
%% Print all Controller Gains

fileID = fopen('Controll_Difference_Equations.dat','w');
fprintf(fileID,' //Channel 1, u1 is Height\n// Channel 2, u2 is Roll\n// Channel 3, u3 is Pitch\n// Channel 4, u4 is Yaw\n');
fprintf(fileID,'\n//Motor 1 PWM input is summed +u1-u3+u4\n// Motor 2 PWM input summed +u1-u2-u4\n// Motor 3 PWM input is summed +u1+u3+u4\n// Motor 4 PWM input summed +u1+u2-u4 \n');
fprintf(fileID,'\n//Height Channel feeds are summed ++++\n// Roll Channel feeds are summed 0-0+\n// Pitch Channel Feeds are summed -0+0\n// Yaw channel feeds are summed +-+- \n');

fprintf(fileID,'\n//The regulator controller with forward-Eulers general formula is:  \n');
fprintf(fileID,'// u(k)= K1*u(k-1)+ K2*r(k-1) + K3*r(k-1) + K4*x1hat(k-1) + K5*x1hat(k-1) + K6*x2hat(k-1) + K7*x3hat(k-1)\n');

fprintf(fileID,'\n//  The regulator controller with backwards-Eulers general formula is:  \n');
fprintf(fileID,'// u(k)= K1*r(k) + K2*v(k) + K3*x1hat(k) + K4*x2hat(k) + K5*x3hat(k)');
fprintf(fileID,'\n fixed kh1 = %12.8f;     fixed kh2 = %12.8f;     fixed kh3 = %12.8f;     fixed kh4 = %12.8f;   fixed kh5 = %12.8f;',     Feed_Forward_Height_d,  1,-K_Height_d(1),   -K_Height_d(2), -K_Height_d(3));
fprintf(fileID,'\n fixed kr1 = %12.8f;     fixed kr2 = %12.8f;     fixed kr3 = %12.8f;     fixed kr4 = %12.8f;   fixed kr5 = %12.8f;',     Feed_Forward_Roll_d,    1,-K_Roll_d(1),     -K_Roll_d(2),   -K_Roll_d(3));
fprintf(fileID,'\n fixed kp1 = %12.8f;     fixed kp2 = %12.8f;     fixed kp3 = %12.8f;     fixed kp4 = %12.8f;   fixed kp5 = %12.8f;',     Feed_Forward_Roll_d,    1,-K_Roll_d(1),     -K_Roll_d(2),   -K_Roll_d(3));
fprintf(fileID,'\n fixed ky1 = %12.8f;     fixed ky2 = %12.8f;     fixed ky3 = %12.8f;     fixed ky4 = %12.8f;   fixed ky5 = %12.8f;\n',   Feed_Forward_Yaw_d,     1,-K_Yaw_d(1),      -K_Yaw_d(2),    -K_Yaw_d(3));

fprintf(fileID,'\n// The output of the backwards-Eulers integrator is:  \n');
fprintf(fileID,'// v(k)= v(k-1) + KiTs*[r(k) - x1hat(k)]\n');
fprintf(fileID,'// This the where the clamping on the output should be applied\n');
fprintf(fileID,'\n fixed khi = %12.8f;',-K_Height_i*Ts);
fprintf(fileID,'\n fixed kri = %12.8f;',-K_Roll_i*Ts);
fprintf(fileID,'\n fixed kpi = %12.8f;',-K_Roll_i*Ts);
fprintf(fileID,'\n fixed kyi = %12.8f;',-K_Yaw_i*Ts);


% fprintf(fileID,'\n\n//The regulator controller with Tustisn general formula is:  \n');
% fprintf(fileID,'// u(k)= K1*u(k-1)+ K2*r(k) + K3*r(k-1) + K4*x1hat(k) + K5*x1hat(k-1) + K6*x2hat(k) + K7*x2hat(k-1) + K8*x3hat(k) + K9*x3hat(k-1)');
% fprintf(fileID,'\n\n// Where\n //Ki is the ith element of the controller matrix for a particular channel (Height, Roll, Pitch, Yaw)\n// r is the setpoint signal \n// u is the channel signal \n// x1hat is the position or angle estimate for that system \n// x2hat is the rate of change of x1 estimate \n// x3hat is the motor state estimate for a channel \n');
% 
% fprintf(fileID,'\n//The controller gains for above controller with tustins method and no clamping is:  \n');
% t=transpose(U_Gains);
% fprintf(fileID,'fixed kh1 = %12.8f;  fixed kh2 = %12.8f; fixed kh3 =  %12.8f; fixed kh4 =  %12.8f; fixed kh5 =  %12.8f; fixed kh6 =  %12.8f; fixed kh7 =  %12.8f; fixed kh8 =  %12.8f; fixed kh9 =  %12.8f;\n',t(1:9));
% fprintf(fileID,'fixed kr1 = %12.8f;  fixed kr2 = %12.8f; fixed kr3 =  %12.8f; fixed kr4 =  %12.8f; fixed kr5 =  %12.8f; fixed kr6 =  %12.8f; fixed kr7 =  %12.8f; fixed kr8 =  %12.8f; fixed kr9 =  %12.8f;\n',t(10:18));
% fprintf(fileID,'fixed kp1 = %12.8f;  fixed kp2 = %12.8f; fixed kp3 =  %12.8f; fixed kp4 =  %12.8f; fixed kp5 =  %12.8f; fixed kp6 =  %12.8f; fixed kp7 =  %12.8f; fixed kp8 =  %12.8f; fixed kp9 =  %12.8f;\n',t(19:27));
% fprintf(fileID,'fixed ky1 = %12.8f;  fixed ky2 = %12.8f; fixed ky3 =  %12.8f; fixed ky4 =  %12.8f; fixed ky5 =  %12.8f; fixed ky6 =  %12.8f; fixed ky7 =  %12.8f; fixed ky8 =  %12.8f; fixed ky9 =  %12.8f;\n',t(28:36));
% % fprintf(fileID,'//%12s %12s %12s %12s %12s %12s %12s %12s %12s\n','k1','k2','k3','k4','k5','k6','k7','k8','k9');
% % fprintf(fileID,'//%12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f\n',transpose(U_Gains));
% % fprintf(fileID,'\n\n//Where 1st row is for channel 1, 2nd for channel 2, etc\n');

fprintf(fileID,'\n\n// The states for each controller must be observed, this means there should be 4 sets of the 3 observer equations below\n');
fprintf(fileID,'\n// The observer general formula is:  \n');
fprintf(fileID,' // x1hat(k+1)= F11*x1hat(k)+ F12*x2hat(k) + F13*x3hat(k) + G11*y1(k) + G12*y2(k) + B1*u(k)\n// x2hat(k+1)= F21*x1hat(k)+ F22*x2hat(k) + F23*x3hat(k) + G21*y1(k) + G22*y2(k) + B2*u(k)\n// x3hat(k+1)= F31*x1hat(k)+ F32*x2hat(k) + F33*x3hat(k) + G31*y1(k) + G32*y2(k) B3*u(k)');
fprintf(fileID,'\n\n// Where\n// x1hat is estimated x1 value for position/angle\n// x2hat is estimated x2 value for postion/angle rate\n// x3hat is estimated x3 value is the motor state\n// y1 is output of position/angle sensor \n// y2 is output of rate sensor\n');


fprintf(fileID,'\n\n// The height observer gains are:  \n');
t=transpose(Height_Obs);
fprintf(fileID,'fixed Fh11 = %12.8f;  fixed Fh12 = %12.8f; fixed Fh13 =  %12.8f;  fixed Gh11 =  %12.8f;  fixed Bh1 =  %12.8f;  \n',t(1:5));
fprintf(fileID,'fixed Fh21 = %12.8f;  fixed Fh22 = %12.8f; fixed Fh23 =  %12.8f;  fixed Gh21 =  %12.8f;  fixed Bh2 =  %12.8f; \n',t(6:10));
fprintf(fileID,'fixed Fh31 = %12.8f;  fixed Fh32 = %12.8f; fixed Fh33 =  %12.8f;  fixed Gh31 =  %12.8f;  fixed Bh3 =  %12.8f; \n',t(11:15));


fprintf(fileID,'\n//The roll observer gains are:  \n');
t=transpose(Roll_Obs);
fprintf(fileID,'fixed Fr11 = %12.8f;  fixed Fr12 = %12.8f; fixed Fr13 =  %12.8f;  fixed Gr11 =  %12.8f;  fixed Gr12 =  %12.8f; fixed Br1 =  %12.8f; \n',t(1:6));
fprintf(fileID,'fixed Fr21 = %12.8f;  fixed Fr22 = %12.8f; fixed Fr23 =  %12.8f;  fixed Gr21 =  %12.8f;  fixed Gr22 =  %12.8f; fixed Br2 =  %12.8f; \n',t(7:12));
fprintf(fileID,'fixed Fr31 = %12.8f;  fixed Fr32 = %12.8f; fixed Fr33 =  %12.8f;  fixed Gr31 =  %12.8f;  fixed Gr32 =  %12.8f; fixed Br3 =  %12.8f; \n',t(13:18));
% fprintf(fileID,'// %12s %12s %12s %12s %12s \n','Fn1','Fn2','Fn3','Gn1','Gn2');
% fprintf(fileID,'// %12.8f %12.8f %12.8f %12.8f %12.8f\n',transpose(Roll_Obs));

fprintf(fileID,'\n// The pitch observer gains are:  \n');
t=transpose(Pitch_Obs);
fprintf(fileID,'fixed Fp11 = %12.8f;  fixed Fp12 = %12.8f; fixed Fp13 =  %12.8f;  fixed Gp11 =  %12.8f;  fixed Gp12 =  %12.8f;  fixed Bp1 =  %12.8f; \n',t(1:6));
fprintf(fileID,'fixed Fp21 = %12.8f;  fixed Fp22 = %12.8f; fixed Fp23 =  %12.8f;  fixed Gp21 =  %12.8f;  fixed Gp22 =  %12.8f;  fixed Bp2 =  %12.8f; \n',t(7:12));
fprintf(fileID,'fixed Fp31 = %12.8f;  fixed Fp32 = %12.8f; fixed Fp33 =  %12.8f;  fixed Gp31 =  %12.8f;  fixed Gp32 =  %12.8f;  fixed Bp3 =  %12.8f; \n',t(13:18));
% fprintf(fileID,'// %12s %12s %12s %12s %12s \n','Fn1','Fn2','Fn3','Gn1','Gn2');
% fprintf(fileID,'// %12.8f %12.8f %12.8f %12.8f %12.8f\n',transpose(Pitch_Obs));

fprintf(fileID,'\n// The yaw observer gains are:  \n');
t=transpose(Yaw_Obs);
fprintf(fileID,'fixed Fy11 = %12.8f;  fixed Fy12 = %12.8f; fixed Fy13 =  %12.8f;  fixed Gy11 =  %12.8f;  fixed Gy12 =  %12.8f; fixed By1 =  %12.8f; \n',t(1:6));
fprintf(fileID,'fixed Fy21 = %12.8f;  fixed Fy22 = %12.8f; fixed Fy23 =  %12.8f;  fixed Gy21 =  %12.8f;  fixed Gy22 =  %12.8f; fixed By2 =  %12.8f;\n',t(7:12));
fprintf(fileID,'fixed Fy31 = %12.8f;  fixed Fy32 = %12.8f; fixed Fy33 =  %12.8f;  fixed Gy31 =  %12.8f;  fixed Gy32 =  %12.8f; fixed By3 =  %12.8f;\n',t(13:18));
% fprintf(fileID,'// %12s %12s %12s %12s %12s \n','Fn1','Fn2','Fn3','Gn1','Gn2');
% fprintf(fileID,'// %12.8f %12.8f %12.8f %12.8f %12.8f\n',transpose(Yaw_Obs));

fprintf(fileID,'\n// I believe the outputs will pass from sensor to Kalman then through observer. \n// The observed states will then be used in controller. \n// Controller Outputs will then be summed together\n');

fprintf(fileID,' //The Positional x-y Control is\n')
fprintf(fileID,'\\r(k)= Kp*e(k)+Ki*i*(k); \n //where e(k) is error and i(k) is integration term')
fprintf(fileID,'\n\\i(k)=i(k-1)+e(k); //Integration Term')
fprintf(fileID,'\nThe sign of the y controller output must be changed')
fprintf(fileID,'\nfixed Kp_x = %12.8f;    fixed Ki_x = %12.8f;    fixed Kd_x = %12.8f;',Kp_x,Ki_x,Kd_x)
fprintf(fileID,'\nfixed Kp_y = %12.8f;    fixed Ki_y = %12.8f;    fixed Kd_y = %12.8f;',Kp_y,Ki_y,Kd_y)


fclose(fileID);
type Controll_Difference_Equations.dat