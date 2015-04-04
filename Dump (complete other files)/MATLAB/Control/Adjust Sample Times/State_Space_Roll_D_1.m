%% State-Space Control of a Quadcopter
% State-Space control of Roll axis using  delta D as the input. Pitch axis
% assumed to be the same by symmetry.
%% Control of Pitch or Roll axis

clear all
clc

linesize = 1;
%% Simulation Parameters
load('Model_Parameters.mat');
Ts = 1/50;
sim_length=10;
Feed_Forward_Roll_d = 1/8;
%% Model Derivation
A = [0, 1, 0
     0, 0, 2*Kf*L/(I_Theta)
     0, 0, -1/Motor_TimeConstant];
 
 B = [0
      0
      1/Motor_TimeConstant];
  
  C= [1 0 0
      0 1 0];
  
  D=[0
     0];

states = {'theta' 'theta_dot' 'omega'};
inputs = {'D'};
outputs = {'theta','theta_dot'};
sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
sys_d = c2d(sys_ss,Ts,'zoh');

%% Controllability and Observability
co = ctrb(sys_d);
ob = obsv(sys_d);

controllability = rank(co);
observability = rank(ob);
 
%% Digital Stae Space System
A_d = sys_d.a;
B_d = sys_d.b;
C_d = sys_d.c;
D_d = sys_d.d;
Csize=size(C_d);

A_i = [A_d,zeros(length(A_d)',1); -C_d(1,:)*Ts,ones(1)];
B_i = [B_d;0];
C_i=[C_d(1,:) 0];

%% System Poles
poles = eig(A_i);

%% LQR Method
Q = [2   0     0     0
     0     0.001     0     0
     0     0     0.001     0
     0     0     0     5];
R = 1;
[K_Roll_Reg] = dlqr(A_i,B_i,Q,R);
K_Roll_i=K_Roll_Reg(end);
K_Roll_d=K_Roll_Reg(1:end-1);

sys_cl = ss(A_i-B_i*K_Roll_Reg,B_i,C_i,0);

%% Observer Design
dPoles = eig(sys_cl);
dPole = max(real(dPoles))^15;

P = [dPole, dPole+.01, dPole+.02];
G_Roll_d = place(A_d',C_d',P)';
%% Pole Placement Design
% zeta = .707;
% w_n = 7;
% 
% p1 = exp(Ts*(-zeta*w_n + sqrt(zeta*w_n^2-w_n^2)));
% p2 = exp(Ts*(-zeta*w_n - sqrt(zeta*w_n^2-w_n^2)));
% p3 = 0.7515 ; % weird motor pole position
% p4 = 0.98; % Integrator Pole
% 
% 
% K_Roll_d = place(A_i,B_i,[p1 p2 p3 p4]);
% 
% sys_cl = ss(A_i-B_i*K_Roll_d,B_i,C_i,0);
%% Simulink
A_Roll_d=A_d;
B_Roll_d=B_d;
C_Roll_d=C_d;
model = 'Roll_Diff_SS_D_1';
sim(model)

%% Plot Responses
figure
subplot(3,1,1)
plot(get(Setpoint,'Time'),get(Setpoint,'Data'),'g')
hold on
plot(Roll);
hold on
%plot(RollD,'r');
%ylim([-10 10])
title('Simulink: Roll');
ylabel('Roll [rad]')
xlabel('Time [seconds]')
grid on
%saveas(gcf, 'Roll_Roll.png')
subplot(3,1,2)
plot(RollRate);
hold on
%plot(RollRateD,'r');
title('Simulink: Roll Rate');
ylabel('Roll Rate [rad/s]');
xlabel('Time [seconds]')
grid on
%saveas(gcf, 'Roll_RollRate.png')
subplot(3,1,3)
plot(Pulse);
title('PWM');
grid on
ylabel('PWM Length')
xlabel('Time [seconds]')
%% Save Controller Parameters
save('Roll_Controller_SS_D_Data','A_Roll_d','B_Roll_d','C_Roll_d','K_Roll_d','Feed_Forward_Roll_d','K_Roll_i','G_Roll_d')