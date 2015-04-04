%% Finding Parameters for PID control of Yaw
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
omega_0=520;            % find a value of omega_0

%% 3rd order Model of Yaw-Duty Cycle Relationship

sys_Yaw = tf([J/I_Psi 2*omega_0*D/I_Psi],[0.07 1 0 0]);
s=tf('s');
%% Open Loop
% Open Loop Step Response
step(sys_Yaw)
figure
impulse(sys_Yaw)
% Root Locus Map
figure
rlocus(sys_Yaw)
axis([-20 10 -15 15])
sgrid
%% Desired Characteristics
Ts= 5;                  % Settling Time
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
angleSys = -imag(evalfr(sys_Yaw,DCL_Pole_1))/real(evalfr(sys_Yaw,DCL_Pole_1));
alpha_c=pi-angleSys;                % Angle between Desired Poles and real axis at introduced zero
zc=ZetaWn+(Wd/tan(alpha_c));        % Introduced Zero Position on -sigma axis
Kd=1/abs((DCL_Pole_1+zc)*evalfr(sys_Yaw,DCL_Pole_1));   % Differential Gain
C_PD=Kd*(s+zc);                        % PD Controller
%% Plotting Result Modified Root Locus
figure
rlocus(C_PD*sys_Yaw)
title('Root Locus with PI Control')
hold on
plot(real(DCL_Pole_1),imag(DCL_Pole_1),'^');
hold on
plot(real(DCL_Pole_2),imag(DCL_Pole_2),'^');
sgrid
%% Plotting Close Loop Step Response
T = feedback(sys_Yaw,C_PD);
figure
step(T)

%% Save to .mat file
save('Pitch_Roll_Controller_Parameters_PD','C_PD','Kd','zc');