%% State-Space Control of a Quadcopter
% Control of Height Model
% State-Space control of Height using differential input delta D as the
% input. Motors assumed identical.

close all       % Close all open figures
%% Load Model Para meters
load('Model_Parameters.mat');
%% Model Derivation
A = [0, 1, 0
     0, 0, 4*Kf/Mass
     0, 0, -1/Motor_TimeConstant];
 
 B = [0
      0
      1/Motor_TimeConstant];
  
  C= [1 0 0];
  
  D=0;

states = {'Height' 'Height_dot' 'omega'};
inputs = {'D'};
outputs = {'Height'};
sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
sys_d = c2d(sys_ss,Ts,'zoh');

%% Controllability and Observability Tests
co = ctrb(sys_d);
ob = obsv(sys_d);

controllability = rank(co);     % Controllable if rank = order of model
observability = rank(ob);       % Observable if rank = order of model
 
%% Convert Model to Digital State Space System
A_d = sys_d.a;
B_d = sys_d.b;
C_d = sys_d.c;
D_d = sys_d.d; 
Csize=size(C_d);

A_i = [A_d,zeros(length(A_d)',1); -C_d(1,:)*Ts,ones(1)];
B_i = [B_d;0];
C_i=[C_d(1,:) 0];

%% Find System Poles
poles = eig(A_i);
%% LQR Controller Design Method
Q = diag([5.5 0.001 0.001 5]);
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
%% Test on Simulink
sim_length=15; 
model = 'Height_Diff_SS_D_1';
sim(model)
%% Plot Responses
figure
subplot(3,1,1)
plot(get(Height_Setpoint_Sim,'Time'),get(Height_Setpoint_Sim,'Data'),'g')
hold on
plot(Height);
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
%% Save Controller Parameters to .mat file
save('Height_Controller_SS_D_Data','A_Height_d','B_Height_d','C_Height_d','K_Height_d','Feed_Forward_Height_d','K_Height_i','G_Height_d')
