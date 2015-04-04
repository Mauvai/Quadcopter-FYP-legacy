clear
close all
clc

%% State-Space Control of a Quadcopter
% State-Space control of Roll axis using  delta D as the input. Pitch axis
% assumed to be the same by symmetry.
%% Control of Pitch or Roll axis
linesize = 1;
%% Simulation Parameters
load('Model_Parameters.mat');
Ts = 30e-3;

omega_0 = 500;

Weight=Mass*g;

%%Simulation Parameters
sim_length = 10;

%% Inital Conditions and Limits
Phi_IC= (0/180)*pi;     % Roll IC
Theta_IC= (5/180)*pi;   % Pitch IC
Psi_IC= (0/180)*pi;     % Yaw IC

PWM_IC=1.4;             % Initialise PWM near region of operation
PWM_LimHigher = 2*inf;      % Limit PWM to Linear region of operation
PWM_LimLower = 1.1*-inf;     % Limit PWM to Linear region of operation

% omega_0 = (0-F_yIntercept)/Kf; %Initialise motors near operating point


omega_Lim = -inf*(0-F_yIntercept)/Kf;
%% Setpoints
Height_Setpoint = 0;
Yaw_Setpoint = 0;
Roll_Setpoint = 0;
Pitch_Setpoint = 0;
%% Model Derivation
A = [-1/Motor_TimeConstant, 0, 0,0,0,0,0,0,0,0,0,0
     0, -1/Motor_TimeConstant, 0,0,0,0,0,0,0,0,0,0
     0, 0, -1/Motor_TimeConstant, 0,0,0,0,0,0,0,0,0
     0, 0, 0, -1/Motor_TimeConstant, 0,0,0,0,0,0,0,0
     0, 0, 0, 0, 0,1,0,0,0,0,0,0
     Kf/Mass, Kf/Mass, Kf/Mass, Kf/Mass, 0,0,0,0,0,0,0,0
     0, 0, 0, 0, 0 ,0,0,1,0,0,0,0
     0, -Kf*L/I_Phi, 0, Kf*L/I_Phi, 0,0,0,0,0,0,0,0
     0, 0, 0, 0, 0,0,0,0,0,1,0,0
     -Kf*L/I_Theta, 0, Kf*L/I_Theta, 0, 0,0,0,0,0,0,0,0
     0, 0, 0, 0, 0,0,0,0,0,0,0,1
     2*Drag*Kw*omega_0/I_Psi, -2*Drag*Kw*omega_0/I_Psi, 2*Drag*Kw*omega_0/I_Psi, -2*Drag*Kw*omega_0/I_Psi, 0,0,0,0,0,0,0,0];
 
B = [1/Motor_TimeConstant,0,0,0
      0,1/Motor_TimeConstant,0,0
      0,0,1/Motor_TimeConstant,0
      0,0,0,1/Motor_TimeConstant
      0,0,0,0
      0,0,0,0
      0,0,0,0
      0,0,0,0
      0,0,0,0
      0,0,0,0
      0,0,0,0
      0,0,0,0];
  
C= [0 0 0 0 1 0 0 0 0 0 0 0
    0 0 0 0 0 0 1 0 0 0 0 0
    0 0 0 0 0 0 0 1 0 0 0 0
    0 0 0 0 0 0 0 0 1 0 0 0
    0 0 0 0 0 0 0 0 0 1 0 0
    0 0 0 0 0 0 0 0 0 0 1 0
    0 0 0 0 0 0 0 0 0 0 0 1];
  
  D=[0];

states = {'Motor1' 'Motor2' 'Motor3' 'Motor4' 'Height' 'Height_dot' 'Roll' 'Roll_dot' 'Pitch' 'Pitch_dot' 'Yaw' 'Yaw_dot'};
inputs = {'u1' 'u2' 'u3' 'u4'};

outputs = {'Height','Roll','Roll_dot','Pitch','Pitch_dot','Yaw','Yaw_dot'};
sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
sys_d = c2d(sys_ss,Ts,'zoh');

%% Controllability and Observability
co = ctrb(sys_d);
ob = obsv(sys_d);

controllability = rank(co);
observability = rank(ob);
 
%% Digital State Space System
A_d = sys_d.a;
B_d = sys_d.b;
C_d = sys_d.c;
D_d = sys_d.d;
Csize=size(C_d);

A_i = [A_d,zeros(length(A_d)',1); -C_d(1,:)*Ts,ones(1)];
B_i = [B_d;0 0 0 0];
C_i=[C_d(1,:) 0];

%% System Poles
poles = eig(A_i);

%% LQR Method with no integrator
Q=diag([1e-6, 1e-6, 1e-6, 1e-6, 0.005, 1e-4, 0.01, 1e-4, 0.01, 1e-4, 0.1, 1e-4]);
R = [1  0   0   0
    0   1   0   0
    0   0   1   0
    0   0   0   1];

[K_MIMO_Reg] = dlqr(A_d,B_d,Q,R);

K_MIMO_d=K_MIMO_Reg;

Feed_Forward_MIMO_d = 2;

sys_cl = ss(A_d-B_d*K_MIMO_Reg,B_d,C_d,0);

%% LQR Method with 1 integrator
% Q = [0.000001  0   0   0   0    0     0     0    0   0  0   0  0
%      0  0.000001  0   0   0    0     0     0    0   0  0   0  0
%      0  0   0   0.000001   0    0     0     0    0   0  0   0  0
%      0  0   0   0   0    0.000001     0     0    0   0  0   0  0
%      0  0   0   0   0.01    0     0     0    0   0  0   0  0
%      0  0   0   0   0    0.000001     0     0    0   0  0   0  0
%      0  0   0   0   0    0     0.1     0    0   0  0   0  0
%      0  0   0   0   0    0     0     0.000001    0   0  0   0  0
%      0  0   0   0   0    0     0     0    0.1   0  0   0  0
%      0  0   0   0   0    0     0     0    0   0.000001  0   0  0
%      0  0   0   0   0    0     0     0    0   0  0.001   0  0
%      0  0   0   0   0    0     0     0    0   0  0   0.000001  0
%      0  0   0   0   0    0     0     0    0   0  0   0  0.000001];
%  
% R = [1  0   0   0
%     0   1   0   0
%     0   0   1   0
%     0   0   0   1];
% 
% [K_MIMO_Reg] = dlqr(A_i,B_i,Q,R);
% 
% K_MIMO_i=K_MIMO_Reg(end);
% K_MIMO_d=K_MIMO_Reg(1:end-1);
% 
% Feed_Forward_MIMO_d = 2;
% 
% sys_cl = ss(A_i-B_i*K_MIMO_Reg,B_i,C_i,0);


%% Minmial Realization
sysr = minreal(sys_d);
%% Observer Design
dPoles = eig(sys_cl);
dPole = max(real(dPoles))^50;

P = [dPole, dPole+.01, dPole+.02,dPole+.03,dPole+.04,dPole+.05,dPole+.06,dPole+.07,dPole+.08,dPole+.09,dPole+.10,dPole+.11];
Pfreq = abs(log(P)/(2*pi)/Ts);
G_MIMO_d = place(A_d',C_d',P)';

A_MIMO_d = A_d;
B_MIMO_d = B_d;
C_MIMO_d = C_d;
%% Simulink
model = 'MIMO_Model_D_R1';
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

% % Channel PWM
% figure(8)
% subplot(2,2,1)
% plot(HeightChannel)
% title('Height Channel')
% ylabel('Pulse Length [ms]')
% xlabel('Time [s]')
% grid on
% subplot(2,2,2)
% plot(YawChannel)
% title('Yaw Channel')
% ylabel('Pulse Length [ms]')
% xlabel('Time [s]')
% grid on
% subplot(2,2,3)
% plot(PitchChannel)
% title('Pitch Channel')
% ylabel('Pulse Length [ms]')
% xlabel('Time [s]')
% grid on
% subplot(2,2,4)
% plot(RollChannel)
% title('Roll Channel')
% ylabel('Pulse Length [ms]')
% xlabel('Time [s]')
% grid on