%% Test the Designed Controllers
% Pitch = State Space Regulator with Precomp
% Roll = State Space Regulator with Precomp
% Yaw = PID
% Height = PID

clc
close all
clear all
%% Model Parameters
load('Model_Parameters')
load('Roll_Controller_SS_D_Data')
load('Height_Controller_SS_D_Data')
load('Yaw_Controller_SS_D_Data')

Weight=Mass*g;

%%Simulation Parameters
sim_length = 30;

%% Inital Conditions and Limits
Phi_IC= (0/180)*pi;     % Roll IC
Theta_IC= (5/180)*pi;   % Pitch IC
Psi_IC= (0/180)*pi;     % Yaw IC

PWM_IC=1.4;             % Initialise PWM near region of operation
PWM_LimHigher = 2;      % Limit PWM to Linear region of operation
PWM_LimLower = 1.1;     % Limit PWM to Linear region of operation

omega_0 = (0-F_yIntercept)/Kf; %Initialise motors near operating point

Ts=1/50;

omega_Lim = (0-F_yIntercept)/Kf;
%% Setpoints
Height_Setpoint = 0;
Yaw_Setpoint = 0;
Roll_Setpoint = 0;
Pitch_Setpoint = 0;
%% Run Test on Simulink\
model='Full_Model_D_R3';
sim(model)

%% Plot
figure(1)
subplot(3,1,1)
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

figure(2)
subplot(3,1,1)
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

figure(3)
subplot(3,1,1)
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

%% Controller Parameters for On-Chip Formulas
F_Height_d = A_Height_d-G_Height_d*C_Height_d;
F_Roll_d = A_Roll_d-G_Roll_d*C_Roll_d;
F_Pitch_d = A_Roll_d-G_Roll_d*C_Roll_d;
F_Yaw_d = A_Yaw_d-G_Yaw_d*C_Yaw_d;
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
% % Names={'Height';
% %      'Roll';
% %      'Pitch';
% %      'Yaw';};
% %  U_Gains=[Names ,U_Gains]

% Height Observer
Height_Obs = [F_Height_d G_Height_d];
% Roll Observer
Roll_Obs = [F_Roll_d G_Roll_d];
% Pitch Observer
Pitch_Obs = [F_Roll_d G_Roll_d];
% Yaw Observer
Yaw_Obs = [F_Roll_d G_Yaw_d];
%% Print all Controller Gains
% fprintf(['Height Regulator Gains are ',num2str(K_Height_d)])
% fprintf(['Height Integrator Gain is ',num2str(K_Height_i),char(10)]);
% fprintf(['Roll Regulator Gains are ',num2str(K_Roll_d)])
% fprintf(['Roll Integrator Gain is ',num2str(K_Roll_i),char(10)])
% fprintf(['Pitch Regulator Gains are ',num2str(K_Roll_d)])
% fprintf(['Pitch Integrator Gain is ',num2str(K_Roll_i),char(10)])
% fprintf(['Yaw Regulator Gains are ',num2str(K_Yaw_d)])
% fprintf(['Pitch Integrator Gain is ',num2str(K_Yaw_i),char(10)])

fileID = fopen('Controll_Difference_Equations.dat','w');
formatSpec = '%s %d %2.1f %s\n';
fprintf(fileID,[' System 1 is Height\n System 2 is Roll\n System 3 is Pitch\n System 4 is Yaw\n']);

fprintf(fileID,['\nThe controller general formula is:  \n']);
fprintf(fileID,[' u(k)= K1*u(k-1)+ K2*r(k) + K3*r(k-1) + K4*x1(k) + K5*x(k-1) + K6*x2(k) + K7*x2(k-1) + K8*x3(k) + K9*x3(k-1)']);
fprintf(fileID,['\nWhere\n Ki is the ith element of the controller matrix for a particular system (Height, Roll, Pitch, Yaw)\n r is the setpoint signal \n u is the control signal \n x1 is the position or angle for that system \n x2 is the rate of change of x1 \n x3 is the motor state for a system\n']);

fprintf(fileID,['\nThe controller gains are:  \n']);
fprintf(fileID,'%12s %12s %12s %12s %12s %12s %12s %12s %12s\n','k1','k2','k3','k4','k5','k6','k7','k8','k9');
fprintf(fileID,'%12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f\n',transpose(U_Gains));


fprintf(fileID,['\nThe observer general formula is:  \n']);
fprintf(fileID,[' xhat1(k+1)= F11*x1hat(k)+ F12*x2hat(k) + F13*x3hat(k) + G11*y1(k) + G12*y2(k) \n xhat2(k+1)= F21*x1hat(k)+ F22*x2hat(k) + F23*x3hat(k) + G21*y1(k) + G22*y2(k) \n xhat3(k+1)= F31*x1hat(k)+ F32*x2hat(k) + F33*x3hat(k) + G31*y1(k) + G32*y2(k) ']);
fprintf(fileID,['\nWhere\n x1hat is estimated x1 value \n x2hat is estimated x2 value \n x3hat is estimated x3 value \n y1 is output of position/angle sensor \n y2 is output of rate sensor\n']);

fprintf(fileID,['\n\nThe height observer gains are:  \n']);
fprintf(fileID,'%12s %12s %12s %12s\n','Fn1','Fn2','Fn3','Gn1');
fprintf(fileID,'%12.8f %12.8f %12.8f %12.8f\n',transpose(Height_Obs));

fprintf(fileID,['\nThe roll observer gains are:  \n']);
fprintf(fileID,'%12s %12s %12s %12s %12s \n','Fn1','Fn2','Fn3','Gn1','Gn2');
fprintf(fileID,'%12.8f %12.8f %12.8f %12.8f %12.8f\n',transpose(Roll_Obs));

fprintf(fileID,['\nThe pitch observer gains are:  \n']);
fprintf(fileID,'%12s %12s %12s %12s %12s \n','Fn1','Fn2','Fn3','Gn1','Gn2');
fprintf(fileID,'%12.8f %12.8f %12.8f %12.8f %12.8f\n',transpose(Pitch_Obs));

fprintf(fileID,['\nThe yaw observer gains are:  \n']);
fprintf(fileID,'%12s %12s %12s %12s %12s \n','Fn1','Fn2','Fn3','Gn1','Gn2');
fprintf(fileID,'%12.8f %12.8f %12.8f %12.8f %12.8f\n',transpose(Yaw_Obs));

% fprintf(fileID,formatSpec,['Height Regulator Gains are ',num2str(K_Height_d),'.',char(10)]);
% fprintf(fileID,formatSpec,['Height Integrator Gain is ',num2str(K_Height_i),'.',char(10)]);
% fprintf(fileID,formatSpec,['Roll Regulator Gains are ',num2str(K_Roll_d),'.',char(10)]);
% fprintf(fileID,formatSpec,['Roll Integrator Gain is ',num2str(K_Roll_i),'.',char(10)]);
% fprintf(fileID,formatSpec,['Pitch Regulator Gains are ',num2str(K_Roll_d),'.',char(10)]);
% fprintf(fileID,formatSpec,['Pitch Integrator Gain is ',num2str(K_Roll_i),'.',char(10)]);
% fprintf(fileID,formatSpec,['Yaw Regulator Gains are ',num2str(K_Yaw_d),'.',char(10)]);
% fprintf(fileID,formatSpec,['Pitch Integrator Gain is ',num2str(K_Yaw_i),'.',char(10)]);
% fprintf(fileID,formatSpec,char(10));
% fprintf(fileID,formatSpec,['HeightChannel',num2str(K_Yaw_i),'.',char(10)]);

fclose(fileID);
type Controll_Difference_Equations.dat
