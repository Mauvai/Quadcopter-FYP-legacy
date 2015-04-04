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
%% First order Model of Pitch-Duty Cycle Relationship
% Theta(s)/Duty(s)= Kf.L/(I_Theta*(1*sTau)s^2)
Motor_Numerator     = [Kf*L];
Motor_Denominator   = I_Theta*[Motor_TimeConstant 1];   
sys_Motor=tf(Motor_Numerator,Motor_Denominator);        % Motor Model
sys_Quad = tf([1],[1 B_Pitch 0]);                       % Quadcopter Model
sys=sys_Quad*sys_Motor;                                 % Full Pitch Model
s=tf('s');
%% Open Loop
% Open Loop Step Response
step(sys)
figure
impulse(sys)
% Root Locus Map
figure
rlocus(sys)
axis([-20 10 -15 15])
sgrid
%% Desired Characteristics
Ts= 1;                  % Settling Time
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
% Evaluate Phase of of Poles
angleSys = -imag(evalfr(sys,DCL_Pole_1))/real(evalfr(sys,DCL_Pole_1));
alpha_c=pi-angleSys;                % Angle between Desired Poles and real axis at introduced zero
zc=ZetaWn+(Wd/tan(alpha_c));        % Introduced Zero Position on -sigma axis
Kd=1/abs((DCL_Pole_1+zc)*evalfr(sys,DCL_Pole_1));   % Differential Gain
C_PD=Kd*(s+zc);                        % PD Controller
%% Plotting Result Modified Root Locus
figure
rlocus(C_PD*sys)
title('Root Locus with PI Control')
hold on
plot(real(DCL_Pole_1),imag(DCL_Pole_1),'^');
hold on
plot(real(DCL_Pole_2),imag(DCL_Pole_2),'^');
sgrid
%% Plotting Close Loop Step Response
T = feedback(sys,C_PD);
figure
step(T)

%% Save to .mat file
save('Pitch_Roll_Controller_Parameters_PD','C_PD','Kd','zc');