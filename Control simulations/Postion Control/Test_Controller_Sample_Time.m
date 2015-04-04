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
load('xy_Controller_PD_D_Data')
%load('xy_Controller_PID_D_Data')
Weight=Mass*g;

%%Simulation Parameters
sim_length = 100;

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
Y_Setpoint = 0;
X_Setpoint = 0;
%% Controller Parameters for On-Chip Formulas
% Height Controller
Height_u = [1 (K_Height_i*Ts+Feed_Forward_Height_d) (K_Height_i*Ts-Feed_Forward_Height_d) (-K_Height_i*Ts+K_Height_d(1)) (-K_Height_i*Ts-K_Height_d(1)) K_Height_d(2) -K_Height_d(2) K_Height_d(3) -K_Height_d(3)];
% Roll Controller
Roll_u = [1 (K_Roll_i*Ts+Feed_Forward_Roll_d) (K_Roll_i*Ts-Feed_Forward_Roll_d) (-K_Roll_i*Ts+K_Roll_d(1)) (-K_Roll_i*Ts-K_Roll_d(1)) K_Roll_d(2) -K_Roll_d(2) K_Roll_d(3) -K_Roll_d(3)];
% Pitch Controller
Pitch_u = [1 (K_Roll_i*Ts+Feed_Forward_Roll_d) (K_Roll_i*Ts-Feed_Forward_Roll_d) (-K_Roll_i*Ts+K_Roll_d(1)) (-K_Roll_i*Ts-K_Roll_d(1)) K_Roll_d(2) -K_Roll_d(2) K_Roll_d(3) -K_Roll_d(3)];
% Yaw Controller
Yaw_u = [1 (K_Yaw_i*Ts+Feed_Forward_Yaw_d) (K_Yaw_i*Ts-Feed_Forward_Yaw_d) (-K_Yaw_i*Ts+K_Yaw_d(1)) (-K_Yaw_i*Ts-K_Yaw_d(1)) K_Yaw_d(2) -K_Yaw_d(2) K_Yaw_d(3) -K_Yaw_d(3)];
%Controller Matrix
U_Gains= [Height_u;
    Roll_u;
    Pitch_u;
    Yaw_u];

F_Height_d = A_Height_d-G_Height_d*C_Height_d;
F_Roll_d = A_Roll_d-G_Roll_d*C_Roll_d;
F_Pitch_d = A_Roll_d-G_Roll_d*C_Roll_d;
F_Yaw_d = A_Yaw_d-G_Yaw_d*C_Yaw_d;

% Height Observer
Height_Obs = [F_Height_d G_Height_d];
% Roll Observer
Roll_Obs = [F_Roll_d G_Roll_d];
% Pitch Observer
Pitch_Obs = [F_Roll_d G_Roll_d];
% Yaw Observer
Yaw_Obs = [F_Roll_d G_Yaw_d];
%% Run Test on Simulink\
model='Full_Model_D_R6';
sim(model)

%% Plot of Non Linear Model Responses
figure(1); subplot(3,1,1)
plot(Height_Setpoint_Sim,'g')
hold on
plot(Height)
title('Height')
ylabel('Height [m]')
xlabel('Time [s]')
grid on
subplot(3,1,2)
plot(X_Setpoint_Sim,'g')
hold on
plot(X_Position)
title('X Position')
ylabel('X Position [m]')
xlabel('Time [s]')
grid on
subplot(3,1,3)
plot(Y_Setpoint_Sim,'g')
hold on
plot(Y_Position)
title('Y Position')
ylabel('Y Position [m]')
xlabel('Time [s]')
grid on
saveas(gcf, 'xyz', 'png')

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
saveas(gcf, 'xyz_rate', 'png')


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
saveas(gcf, 'rpy', 'png')


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
saveas(gcf, 'rpy_rate', 'png')


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
saveas(gcf, 'MotorSpeedResponses', 'png')


%For Duty Cycles When I get chance
figure(6)
subplot(2,2,1)
stairs(get(PWM1,'time'),get(PWM1,'data'))%plot(PWM1)
title('PWM1')
xlabel('Time [s]')
ylabel('Pulse Length [ms]')
grid on
subplot(2,2,2)
stairs(get(PWM2,'time'),get(PWM2,'data'))%plot(PWM2)
title('PWM2')
ylabel('Pulse Length [ms]')
xlabel('Time [s]')
grid on
subplot(2,2,3)
stairs(get(PWM3,'time'),get(PWM3,'data'))%plot(PWM3)
title('PWM3')
ylabel('Pulse Length [ms]')
xlabel('Time [s]')
grid on
subplot(2,2,4)
stairs(get(PWM4,'time'),get(PWM4,'data'))%plot(PWM4)
title('PWM4')
ylabel('Pulse Length [ms]')
xlabel('Time [s]')
grid on
saveas(gcf, 'PWMResponses', 'png')


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
saveas(gcf, 'ThrustResponses', 'png')


% Channel PWM
figure(8)
subplot(2,2,1)
stairs(get(HeightChannel,'time'),get(HeightChannel,'data'))
title('Height Channel')
ylabel('Pulse Length [ms]')
xlabel('Time [s]')
grid on
subplot(2,2,2)
stairs(get(YawChannel,'time'),get(YawChannel,'data'))
%plot(YawChannel)
title('Yaw Channel')
ylabel('Pulse Length [ms]')
xlabel('Time [s]')
grid on
subplot(2,2,3)
stairs(get(PitchChannel,'time'),get(PitchChannel,'data'))
%plot(PitchChannel)
title('Pitch Channel')
ylabel('Pulse Length [ms]')
xlabel('Time [s]')
grid on
subplot(2,2,4)
stairs(get(RollChannel,'time'),get(RollChannel,'data'))
%plot(RollChannel)
title('Roll Channel')
ylabel('Pulse Length [ms]')
xlabel('Time [s]')
grid on
saveas(gcf, 'ChannelResponses', 'png')
