%% Finding Parameters for PID control of Height
% Pitch and roll assumed symmetrical.

% PD Controller Design Youtube Video
% https://www.youtube.com/watch?v=LLms-ei-mrc
%% Clear Workspace, Plots and Command Window
clc
clear
close all
%% Load Physical Parameters from Tests
parameters=('Model_Parameters');
load(parameters)

%% Initial Conditions
omega_0 = 1.7;%(480-Speed_yIntercept) - Kw;

%% First order Model of Height-Duty Cycle Relationship
% Theta(s)/Duty(s)= Kf.L/(I_Theta*(1*sTau)s^2)
Motor_Numerator     =  4*Kf;
Motor_Denominator   = [Motor_TimeConstant 1];   
sys_Motor=tf(Motor_Numerator,Motor_Denominator);        % Motor Model
s=tf('s');
sys=(1/s^2)*sys_Motor;                                 % Full Height Model
%% Root Locus Map
figure
rlocus(sys)
axis([-20 10 -15 15])
sgrid
%% Desired Characteristics
Ts=2;%0.8;                  % Settling Time
Tp=pi/5;                % Time to Peak
Wd=pi/Tp;               % Damped Natural Frequency
ZetaWn = 4/Ts;          % Damping*Natural Frequency
% Desired Close Loop Poles
DCL_Pole_1 = -ZetaWn + 1i*Wd;       % ZetaWn +jWd
DCL_Pole_2 = -ZetaWn - 1i*Wd;       % ZetaWn +jWd
% Plot Desired CL Poles on Root Locus
hold on
plot(real(DCL_Pole_1),imag(DCL_Pole_1),'^');
hold on
plot(real(DCL_Pole_2),imag(DCL_Pole_2),'^');


% Evaluate Magnitude and Phase of of Poles
angleSys = -imag(evalfr(sys,DCL_Pole_1))/real(evalfr(sys,DCL_Pole_1)) - -imag(evalfr(s,DCL_Pole_1))/real(evalfr(s,DCL_Pole_1));
v=pi-angleSys;                % Angle between Desired Poles and real axis at introduced zero
T_I = (1/(2*v))*((imag(DCL_Pole_1)/2)-(real(DCL_Pole_1)*v));
Kc=1/abs((DCL_Pole_1+(2/T_I))^2/(DCL_Pole_1)*evalfr(sys,DCL_Pole_1));   % Controller Gain
C=Kc*( s+(2/T_I) )^2/s;
Ki =T_I;
T_d = T_I/4;
K = Kc/T_d;

%% Plotting Result Modified Root Locus
figure
rlocus(C*sys)
title('Root Locus with PI Control')
hold on
plot(real(DCL_Pole_1),imag(DCL_Pole_1),'^');
hold on
plot(real(DCL_Pole_2),imag(DCL_Pole_2),'^');
sgrid
%% Plot Step Response
T=feedback(C*sys,1);
figure
step(T)

%% Save to .mat file
Kp_Height = K;
Ki_Height = K/T_I;
Kd_Height = K*T_d;
T_I_Height=T_I;
save('Height_Controller_PID','Kp_Height','Ki_Height','Kd_Height','T_I_Height');

%% Simulink
model='HeightModel';
sim(model)

figure
plot(Setpoint,'g')
hold on
plot(Height);
title('Simulink: Height');
ylabel('Height [m]')
xlabel('Time [seconds]')
grid on
saveas(gcf, 'Height_Height.png')

figure
plot(Height_Rate);
title('Simulink: Height Rate');
ylabel('Height Rate[m/s]')
xlabel('Time [seconds]')
grid on
saveas(gcf, 'Height_Rate.png')

figure
plot(Thrust);
title('Simulink: Thrust');
ylabel('Thrust [N]');
xlabel('Time [seconds]')
grid on
saveas(gcf, 'Height_Thrust.png')

figure
plot(Speed);
title('Simulink: Motor Speed');
ylabel('Speed [rpm]')
xlabel('Time [seconds]')
grid on
saveas(gcf, 'Height_Speed.png')

figure
plot(Pulse);
title('Simulink: Pulse Length');
ylabel('Pulse Length [ms]')
xlabel('Time [seconds]')
grid on
saveas(gcf, 'Height_PWM.png')