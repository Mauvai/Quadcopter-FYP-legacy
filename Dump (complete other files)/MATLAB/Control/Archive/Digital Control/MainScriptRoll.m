%% Modelling Pitch
% FYP
% Produce Step Response for pitch adjustment scheme created in simulink for
% a quad-copter
%% Clear Initial
clear
close all
clc

%% Load System Model
model= 'Pitch_Roll_Control';
load_system(model);

motor_number = 2;               %Number of motors
MotorTau = 0.070;               %Time constant of motor
MotorSlope = 0.0189;            %
L = 0.348;                      %Distance between motor and centre of gravity
ITheta = 0.0435;                %Second Moment of Inertia
Kd = 6;                         %Derivative Gain
Ki = 6;                         %Integral Gain
Kp = 12;                        %Proportional Gain
Mass = 2.26;                    %Quad Mass

sim(model)                      %Run Simulation

%% Produce Theta Response
figure
plot(Theta);
pause
%plot(Height);