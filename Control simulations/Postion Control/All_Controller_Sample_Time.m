clear all
%% Initial Set-Up Parameters
close all
clc
Ts=20e-3;
Ts2=1;
%PropDelay = 45e-3;
PropDelay = 0;
%NoiseDelay = 40;
NoiseDelay = 0;
NoiseMean = 0;
%NoiseHeightVariance = 5e-3;
NoiseHeightVariance = 0;
%NoiseAngleVariance = 0.27e-3;
NoiseAngleVariance = 0;
%NoiseAngleRateVariance = 0.1264e-5;
NoiseAngleRateVariance = 0;

ViscousDamping = 36*sin(20/180*pi)/8; %B=Max F in horizontal/max velocity
%% Filter Data
Tau_filter1 = 0.5;      % Time Constant of Cillians Complementary Filter
%% Controller Design Files to retune controllers

 run('Roll_Controller_Sample_Time')
 run('Height_Controller_Sample_Time')
 run('Yaw_Controller_Sample_Time')     
 run('xy_Controller_Sample_Time')   % PI Controller 
 run('xy_Controller_Sample_Time_2')       % PID Controller
%% Controller Test on Non-Linear Model
run('Test_Controller_Sample_Time')
%run('Visual_Controller_Sample_Time')   % Visualization of movement of quad
%in Test_Controller_Sample_Time in 3-D

%% Save Controller gains to text file for Simon
run('Script_Controller_Sample_Time')