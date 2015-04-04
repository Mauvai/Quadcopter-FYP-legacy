%% Test the Designed Controllers
% Pitch = State Space Regulator with Precomp
% Roll = State Space Regulator with Precomp
% Yaw = PID
% Height = PID

clc
close all
clear all
%% Model Parameters
load('Parameters')
load('Yaw_Controller')
load('Roll_Controller_SS_C_Data')
load('Height_Controller_PID')
load('Yaw_Controller_SS_Data.mat')
I_Phi = I_Gamma;


Weight=Mass*g;


K=[1.0000    1.2904    0]
K_Roll_C=[0.3859    0.1652    0]

%%Simulation Parameters
sim_length = 20;

%% Inital Conditions and Limits
Phi_IC= (2/180)*pi;
Theta_IC= (2/180)*pi;
Psi_IC= (10/180)*pi;
PWM_IC=1.4;
omega_0 = (5.6-F_yIntercept)/Kf;
PWM_LimHigher = 2;
PWM_LimLower = 1.1;

omega_Lim = (0-F_yIntercept)/Kf;
%% Setpoints
Height_Setpoint = 0;
Yaw_Setpoint = 0;
Roll_Setpoint = 0;
Pitch_Setpoint = 0;
%% Run Test on Simulink\
model='Full_Model_R1';
sim(model)

%% Plot
figure(1)
subplot(3,1,1)
plot(Height_Setpoint_Sim,'g')
hold on
plot(Height)
title('Height')
grid on
subplot(3,1,2)
plot(X_Position)
title('X')
grid on
subplot(3,1,3)
plot(Y_Position)
title('Y')
grid on

figure(2)
subplot(3,1,1)
plot(HeightRate)
title('Height Rate')
grid on
subplot(3,1,2)
plot(XRate)
title('X Rate')
grid on
subplot(3,1,3)
plot(YRate)
title('Y Rate')
grid on

figure(3)
subplot(3,1,1)
plot(Roll)
title('Roll')
grid on
subplot(3,1,2)
plot(Pitch)
title('Pitch')
grid on
subplot(3,1,3)
plot(Yaw)
title('Yaw')
grid on


figure(4)
subplot(3,1,1)
plot(get(RollRate,'Time'),get(RollRate*180/pi,'Data'))
title('Roll Rate')
grid on
subplot(3,1,2)
plot(get(PitchRate,'Time'),get(PitchRate*180/pi,'Data'))

title('Pitch Rate')
grid on
subplot(3,1,3)
plot(get(YawRate,'Time'),get(YawRate*180/pi,'Data'))
title('Yaw Rate')
grid on


figure(5)
subplot(2,2,1)
plot(MotorSpeed1)
title('Motor Speed 1')
ylabel('Speed [rad/s]')
grid on
subplot(2,2,2)
plot(MotorSpeed2)
title('Motor Speed 2')
ylabel('Speed [rad/s]')
grid on
subplot(2,2,3)
plot(MotorSpeed3)
title('Motor Speed 3')
ylabel('Speed [rad/s]')
grid on
subplot(2,2,4)
plot(MotorSpeed4)
title('Motor Speed 4')
ylabel('Speed [rad/s]')
grid on

%For Duty Cycles When I get chance
figure(6)
subplot(2,2,1)
plot(PWM1)
title('PWM1')
ylabel('Pulse Length')
grid on
subplot(2,2,2)
plot(PWM2)
title('PWM2')
ylabel('Pulse Length')
grid on
subplot(2,2,3)
plot(PWM3)
title('PWM3')
ylabel('Pulse Length')
grid on
subplot(2,2,4)
plot(PWM4)
title('PWM4')
ylabel('Pulse Length')
grid on

% Thrust Forces
figure(7)
subplot(3,1,1)
plot(TotalThrust)
title('Total Thrust')
ylabel('Total Thrust [N]')
grid on
subplot(3,1,2)
plot(MotorThrust)
title('Motor Thrust')
ylabel('Motor Thrust [N]')
grid on
subplot(3,1,3)
plot(VerticalThrust)
title('Vertical Thrust')
ylabel('Vertical Thrust [N]')
grid on

% Channel PWM
figure(8)
subplot(2,2,1)
plot(HeightChannel)
title('Height Channel')
ylabel('Pulse Length')
grid on
subplot(2,2,2)
plot(YawChannel)
title('Yaw Channel')
ylabel('Pulse Length')
grid on
subplot(2,2,3)
plot(PitchChannel)
title('Pitch Channel')
ylabel('Pulse Length')
grid on
subplot(2,2,4)
plot(RollChannel)
title('Roll Channel')
ylabel('Pulse Length')
grid on