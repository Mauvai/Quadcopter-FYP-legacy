%% State-Space Control of a Quadcopter
% State-Space control of Roll axis using  delta D as the input. Pitch axis
% assumed to be the same by symmetry.
%% Control of Pitch or Roll axis

close all
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
     0, 0, 4*Kf/Mass
     0, 0, -1/Motor_TimeConstant];
 
 B = [0
      0
      1/Motor_TimeConstant];
  
  C= [1 0 0];
  
  D=[0];

states = {'Height' 'Height_dot' 'omega'};
inputs = {'D'};
outputs = {'Height'};
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
B_i = [B_d;0];
C_i=[C_d(1,:) 0];

%% System Poles
poles = eig(A_i);

%% LQR Method
Q = [4   0     0     0
     0     0.001     0     0
     0     0     0.001     0
     0     0     0     5];
R = 1;
[K_Height_Reg] = dlqr(A_i,B_i,Q,R);

K_Height_i=K_Height_Reg(end);
K_Height_d=K_Height_Reg(1:end-1);

Feed_Forward_Height_d = 2;

sys_cl = ss(A_i-B_i*K_Height_Reg,B_i,C_i,0);

%% Observer Design
dPoles = eig(sys_cl);
dPole = max(real(dPoles))^50;

P = [dPole, dPole+.01, dPole+.02];
Pfreq = abs(log(P)/(2*pi)/Ts);
G_Height_d = place(A_d',C_d',P)';

A_Height_d = A_d;
B_Height_d = B_d;
C_Height_d = C_d;


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
% K_Height_d = place(A_i,B_i,[p1 p2 p3 p4]);
% 
% sys_cl = ss(A_i-B_i*K_Height_d,B_i,C_i,0);
%% Simulink
model = 'Height_Diff_SS_D_1';
sim(model)
%% Plot Responses
figure
subplot(3,1,1)
plot(get(Height_Setpoint_Sim,'Time'),get(Height_Setpoint_Sim,'Data'),'g')
hold on
plot(Height);
%ylim([-10 10])
title('Simulink: Height');
ylabel('Height [m]')
xlabel('Time [seconds]')
grid on
%saveas(gcf, 'Height_Height.png')
subplot(3,1,2)
plot(HeightRate);
hold on
%plot(HeightRateD,'r');
title('Simulink: Height Rate');
ylabel('Height Rate [m/s]');
xlabel('Time [seconds]')
grid on
%saveas(gcf, 'Height_HeightRate.png')
subplot(3,1,3)
plot(Pulse);
title('PWM');
grid on
ylabel('PWM Length')
xlabel('Time [seconds]')
%% Save Controller Parameters
save('Height_Controller_SS_D_Data','A_Height_d','B_Height_d','C_Height_d','K_Height_d','Feed_Forward_Height_d','K_Height_i','G_Height_d')