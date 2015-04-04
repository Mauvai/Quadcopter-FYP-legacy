%% Finding Parameters for PID control of Height
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

%% First order Model of Height-Duty Cycle Relationship
% Theta(s)/Duty(s)= Kf.L/(I_Theta*(1*sTau)s^2)
Motor_Numerator     =  4*Kf;
Motor_Denominator   = [Motor_TimeConstant 1];   
sys_Motor=tf(Motor_Numerator,Motor_Denominator);        % Motor Model
s=tf('s');
sys=(1/s^2)*sys_Motor;                                 % Full Height Model
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
Ts = 1;                  % Settling Time
Tp =pi/5;                % Time to Peak
PO = 0.3;
Wd =pi/Tp;               % Damped Natural Frequency
ZetaWn = 4/Ts;           % Damping*Natural Frequency

% Desired Close Loop Poles
DCL_Pole_1 = -ZetaWn + 1i*Wd;       % ZetaWn +jWd
DCL_Pole_2 = -ZetaWn - 1i*Wd;       % ZetaWn +jWd
% Plot Desired CL Poles on Root Locus
hold on
plot(real(DCL_Pole_1),imag(DCL_Pole_1),'^');
hold on
plot(real(DCL_Pole_2),imag(DCL_Pole_2),'^');
 