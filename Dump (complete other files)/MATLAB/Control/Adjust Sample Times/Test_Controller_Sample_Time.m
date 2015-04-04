%% Test the Designed Controllers
% Pitch = State Space Regulator with Integrator and Feed Forward gain
% Roll = State Space Regulator with Integrator and Feed Forward gain
% Yaw = State Space Regulator with Integrator and Feed Forward gain
% Height = State Space Regulator with Integrator and Feed Forward gain
close all
%% Model Parameters
load('Model_Parameters')
load('Roll_Controller_SS_D_Data')
load('Height_Controller_SS_D_Data')
load('Yaw_Controller_SS_D_Data')
Weight=Mass*g;

%%Simulation Parameters
sim_length = 60;

%% Inital Conditions and Limits
Phi_IC= (0/180)*pi;     % Roll IC
Theta_IC= (5/180)*pi;   % Pitch IC
Psi_IC= (0/180)*pi;     % Yaw IC

PWM_IC=1.4;             % Initialise PWM near region of operation
PWM_LimHigher = 2;      % Limit PWM to Linear region of operation
PWM_LimLower = 1.1;     % Limit PWM to Linear region of operation

omega_0 = (0-F_yIntercept)/Kf; %Initialise motors near operating point


omega_Lim = (0-F_yIntercept)/Kf;
%% Setpoints
Height_Setpoint = 0;
Yaw_Setpoint = 0;
Roll_Setpoint = 0;
Pitch_Setpoint = 0;
%% Run Test on Simulink\
model='Full_Model_D_R4';
sim(model)

%% Plot
figure(1); subplot(3,1,1)
plot(Height_Setpoint_Sim,'g')
hold on
plot(Height)
title('Height')
ylabel('Height [m]')
xlabel('Time [s]')
grid on
subplot(3,1,2)
plot(X_Position)
title('X Position')
ylabel('X Position [m]')
xlabel('Time [s]')
grid on
subplot(3,1,3)
plot(Y_Position)
title('Y Position')
ylabel('Y Position [m]')
xlabel('Time [s]')
grid on

figure(2); subplot(3,1,1)
plot(HeightRate)
title('Height Rate')
ylabel('Height Rate [m/s]')
xlabel('Time [s]')
grid on
subplot(3,1,2)
plot(XRate)
title('X Rate')
ylabel('X Rate [m/s]')
xlabel('Time [s]')
grid on
subplot(3,1,3)
plot(YRate)
title('Y Rate')
ylabel('Y Rate [m/s]')
xlabel('Time [s]')
grid on

figure(3); subplot(3,1,1)
plot(get(Roll_Setpoint_Sim,'Time'),get(Roll_Setpoint_Sim*180/pi,'Data'),'g')
hold on
plot(get(Roll,'Time'),get(Roll*180/pi,'Data'))
title('Roll Angle')
ylabel('Roll Angle [^{o}]')
xlabel('Time [s]')
grid on
subplot(3,1,2)
plot(get(Pitch,'Time'),get(Pitch*180/pi,'Data'))
hold on
plot(get(Pitch_Setpoint_Sim,'Time'),get(Pitch_Setpoint_Sim*180/pi,'Data'),'g')
title('Pitch Angle')
ylabel('Pitch Angle [^{o}]')
xlabel('Time [s]')
grid on
subplot(3,1,3)
plot(get(Yaw,'Time'),get(Yaw*180/pi,'Data'))
hold on
plot(get(Yaw_Setpoint_Sim,'Time'),get(Yaw_Setpoint_Sim*180/pi,'Data'),'g')
title('Yaw Angle')
ylabel('Yaw Angle [^{o}]')
xlabel('Time [s]')
grid on


figure(4)
subplot(3,1,1)
plot(get(RollRate,'Time'),get(RollRate*180/pi,'Data'))
title('Roll Rate')
ylabel('Roll Rate [^{o}/s]')
xlabel('Time [s]')
grid on
subplot(3,1,2)
plot(get(PitchRate,'Time'),get(PitchRate*180/pi,'Data'))
title('Pitch Rate')
ylabel('PitchRate [^{o}/s]')
xlabel('Time [s]')
grid on
subplot(3,1,3)
plot(get(YawRate,'Time'),get(YawRate*180/pi,'Data'))
title('Yaw Rate')
ylabel('Yaw Rate [^{o}/s]')
xlabel('Time [s]')
grid on


figure(5)
subplot(2,2,1)
plot(MotorSpeed1)
title('Motor Speed 1')
ylabel('Speed [rad/s]')
xlabel('Time [s]')
grid on
subplot(2,2,2)
plot(MotorSpeed2)
title('Motor Speed 2')
ylabel('Speed [rad/s]')
xlabel('Time [s]')
grid on
subplot(2,2,3)
plot(MotorSpeed3)
title('Motor Speed 3')
ylabel('Speed [rad/s]')
xlabel('Time [s]')
grid on
subplot(2,2,4)
plot(MotorSpeed4)
title('Motor Speed 4')
ylabel('Speed [rad/s]')
xlabel('Time [s]')
grid on

%For Duty Cycles When I get chance
figure(6)
subplot(2,2,1)
plot(PWM1)
title('PWM1')
xlabel('Time [s]')
ylabel('Pulse Length [ms]')
grid on
subplot(2,2,2)
plot(PWM2)
title('PWM2')
ylabel('Pulse Length [ms]')
xlabel('Time [s]') 
grid on 
subplot(2,2,3)
plot(PWM3)
title('PWM3')
ylabel('Pulse Length [ms]')
xlabel('Time [s]')
grid on
subplot(2,2,4)
plot(PWM4)
title('PWM4')
ylabel('Pulse Length [ms]')
xlabel('Time [s]')
grid on

% Thrust Forces
figure(7)
subplot(3,1,1)
plot(TotalThrust)
title('Total Thrust')
ylabel('Total Thrust [N]')
xlabel('Time [s]')
grid on
subplot(3,1,2)
plot(MotorThrust3)
title('Motor 3 Thrust')
ylabel('Motor Thrust [N]')
xlabel('Time [s]')
grid on
subplot(3,1,3)
plot(VerticalThrust)
title('Vertical Thrust')
ylabel('Vertical Thrust [N]')
xlabel('Time [s]')
grid on

% Channel PWM
figure(8)
subplot(2,2,1)
plot(HeightChannel)
title('Height Channel')
ylabel('Pulse Length [ms]')
xlabel('Time [s]')
grid on
subplot(2,2,2)
plot(YawChannel)
title('Yaw Channel')
ylabel('Pulse Length [ms]')
xlabel('Time [s]')
grid on
subplot(2,2,3)
plot(PitchChannel)
title('Pitch Channel')
ylabel('Pulse Length [ms]')
xlabel('Time [s]')
grid on
subplot(2,2,4)
plot(RollChannel)
title('Roll Channel')
ylabel('Pulse Length [ms]')
xlabel('Time [s]')
grid on