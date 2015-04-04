%% Modelling Pitch
% FYP
% Produce Step Response for pitch adjustment scheme created in simulink for
% a quad-copter
%% Clear Initial
clear
close all
clc

%% Parameters
parameters=('Parameters');
load(parameters)

motor_number = 2;               %Number of motors
MotorTau = 0.070;               %Time constant of motor

%% Load Full System Model
model= 'Pitch_Roll_Control';
load_system(model); 

Kd = 6;                         %Derivative Gain
Ki = 6;                         %Integral Gain
Kp = 12;                        %Proportional Gain

sim(model)                      %Run Simulation
%% Produce Theta Response
figure
plot(Theta);
pause

%plot(Height);