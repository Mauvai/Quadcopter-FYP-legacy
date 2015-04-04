%% Finding Parameters for PID control of Pitch/Roll
% Pitch and roll assumed symmetrical.

% PD Controller Design Youtube Video
% https://www.youtube.com/watch?v=LLms-ei-mrc
%% Clear Workspace, Plots and Command Window
clc
clear
close all
%% Load Physical Parameters from Tests
parameters=('Parameters');
load(parameters)
%J=0;
omega_0=520;
%% 3rd order Model of Yaw-Duty Cycle Relationship
Motor_Numerator     = [4*Kw];
Motor_Denominator   = [Motor_TimeConstant 1];   
sys_Motor=tf(Motor_Numerator,Motor_Denominator);        % Motor Model
sys_Quad = tf([J,2*D*omega_0],[I_Psi 0 0]);                       % Quadcopter Model
sys_Yaw=sys_Quad*sys_Motor;  
%sys_Yaw = tf([Kw*J/I_Psi Kw*2*omega_0*D/I_Psi],[0.07 1 0 0]);
s=tf('s');

%% Desired Characteristics
Zeta = 0.9;
Wd = 10;
Tp=pi/(2*Wd);
Ts= 4.6*Tp/(pi*Zeta);
ZetaWn=4.6/Ts;

% Ts=3.4;%0.8;                  % Settling Time
% Tp= 2;%pi/5;                % Time to Peak
% Wd=pi/Tp;               % Damped Natural Frequency
% ZetaWn = 4.6/Ts;          % Damping*Natural Frequency


% Desired Close Loop Poles
DCL_Pole_1 = -ZetaWn + 1i*Wd;       % ZetaWn +jWd
DCL_Pole_2 = -ZetaWn - 1i*Wd;       % ZetaWn +jWd
% % Plot Desired CL Poles on Root Locus
% hold on
% plot(real(DCL_Pole_1),imag(DCL_Pole_1),'^');
% hold on
% plot(real(DCL_Pole_2),imag(DCL_Pole_2),'^');


% Evaluate Magnitude and Phase of of Poles
angleSys = -imag(evalfr(sys_Yaw,DCL_Pole_1))/real(evalfr(sys_Yaw,DCL_Pole_1)) - -imag(evalfr(s,DCL_Pole_1))/real(evalfr(s,DCL_Pole_1));
v=pi-angleSys;                % Angle between Desired Poles and real axis at introduced zero
T_I = (1/(2*v))*((imag(DCL_Pole_1)/2)-(real(DCL_Pole_1)*v));
Kc=1/abs((DCL_Pole_1+(2/T_I))^2/(DCL_Pole_1)*evalfr(sys_Yaw,DCL_Pole_1));   % Controller Gain
C=Kc*( s+(2/T_I) )^2/s;
Ki =T_I;
T_d = T_I/4;
K = Kc/T_d;

%% Plotting Result Modified Root Locus
figure
rlocus(C*sys_Yaw)
title('Root Locus with PI Control')
hold on
plot(real(DCL_Pole_1),imag(DCL_Pole_1),'^');
hold on
plot(real(DCL_Pole_2),imag(DCL_Pole_2),'^');
sgrid
% Plot Desired CL Poles on Root Locus
hold on
plot(real(DCL_Pole_1),imag(DCL_Pole_1),'^');
hold on
plot(real(DCL_Pole_2),imag(DCL_Pole_2),'^');
grid


figure
T=feedback(C*sys_Yaw,1);
step(T)

%% Save to .mat file
Kp_Yaw = K;
Ki_Yaw = K/T_I;
Kd_Yaw = K*T_d;
save('Yaw_Controller','Kp_Yaw','Ki_Yaw','Kd_Yaw');

%% Simulink
model='YawModel_R1';
sim(model)

figure
plot(Setpoint,'g')
hold on
plot(Yaw);
%ylim([-10 10])
title('Simulink: Yaw');
ylabel('Yaw [rad]')
xlabel('Time [seconds]')
grid on
saveas(gcf, 'Yaw_Yaw.png')
figure
plot(YawRate);
title('Simulink: Yaw Rate');
ylabel('Yaw Rate [rad/s]');
xlabel('Time [seconds]')
grid on
saveas(gcf, 'Yaw_Yaw.png')
figure
plot(Pulse);
title('Simulink: Pulse Length');
ylabel('Pulse Length [ms]')
xlabel('Time [seconds]')
grid on
saveas(gcf, 'Yaw_PWM.png')