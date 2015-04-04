clear
%%
close all
clc
Ts=20e-3;
PropDelay = 0%45e-3;
NoiseDelay = 0;
NoiseMean = 0;
NoiseHeightVariance = 5e-3;
NoiseAngleVariance = 0%0.05;
NoiseAngleRateVariance = 0.1264e-5;

ViscousDamping = 36/10; %B=F/v

%% Filter Data
Tau_filter1 = 0.5;
%%
run('Roll_Controller_Sample_Time')
run('Height_Controller_Sample_Time')
run('Yaw_Controller_Sample_Time')
load('Roll_Controller_SS_D_Data')
load('Height_Controller_SS_D_Data')
load('Yaw_Controller_SS_D_Data')
run('Test_Controller_Sample_Time')
run('Script_Controller_Sample_Time')
%run('Visual_Sample_Time')
