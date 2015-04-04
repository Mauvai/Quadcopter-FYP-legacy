clear all
close all
clc
Ts=20e-3;
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

ViscousDamping = 0%36*sin(45/180*pi)/10;
Tau_filter1 = 0.5;
%%

% run('Roll_Controller_Sample_Time')
% run('Height_Controller_Sample_Time')
% run('Yaw_Controller_Sample_Time')
% run('xy_Controller_Sample_Time')
%% Test
run('Test_Controller_Sample_Time')

% run('Visual_Controller_Sample_Time')
% run('Script_Controller_Sample_Time')