%% Control of Yaw axis
% State-Space control of Yaw axis using  delta D as the input.
close all
linesize = 1;
%% Load Model Parameters
load('Model_Parameters.mat');
omega_0=520;

YawStep=1;
%% Model Derivation
A = [0, 1, 0
     0, 0, (4*Kw/I_Psi)*( (2*omega_0*Drag) - J)
     0, 0, -1/Motor_TimeConstant];
 
B = [0
     4*Kw*J/(Motor_TimeConstant*I_Psi)
     1/Motor_TimeConstant];
  
C= [1 0 0;
      0 1 0];
  
D=[0;
   0];

states = {'yaw' 'yaw_dot' 'omega'};
inputs = {'D'};
outputs = {'yaw','yaw_dot'};
sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
sys_d = c2d(sys_ss,Ts,'zoh');


%% Controllability and Observability Tests
co = ctrb(sys_ss);
ob = obsv(sys_ss);

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
%% Find System Poles
poles = eig(A_i);

%% LQR Design Method and Feed Forward Gain
Q=diag([12 0.01 0.001 75]);
R = 55;
[K_Yaw_Reg] = dlqr(A_i,B_i,Q,R);
K_Yaw_i=K_Yaw_Reg(end);
K_Yaw_d=K_Yaw_Reg(1:end-1);

Feed_Forward_Yaw_d = 0;

sys_cl = ss(A_i-B_i*K_Yaw_Reg,B_i,C_i,0);
%% Observer Design
dPoles = eig(sys_cl);
dPole = max(real(dPoles))^15;

P = [dPole, dPole+.01, dPole+.02];
G_Yaw_d = place(A_d',C_d',P)';

A_Yaw_d = A_d;
B_Yaw_d = B_d;
C_Yaw_d = C_d;
%% Simulink of Yaw
model='Yaw_Diff_SS_D_2';
sim_length=30;
sim(model)
%% Plots
figure
subplot(3,1,1)
plot(get(Setpoint,'Time'),get(Setpoint,'Data'),'g')
hold on
plot(Yaw);
%ylim([-10 10])
title('Simulink: Yaw');
ylabel('Yaw [rad]')
xlabel('Time [seconds]')
grid on
%saveas(gcf, 'Yaw_Yaw.png')
%figure
subplot(3,1,2)
plot(YawRate);
title('Simulink: Yaw Rate');
ylabel('Yaw Rate [rad/s]');
xlabel('Time [seconds]')
grid on
%saveas(gcf, 'Yaw_Yaw.png')
%figure
subplot(3,1,3)
plot(Pulse);
title('PWM');
grid on
%% Save Controller Parameters
save('Yaw_Controller_SS_D_Data','A_Yaw_d','B_Yaw_d','C_Yaw_d','K_Yaw_d','Feed_Forward_Yaw_d','K_Yaw_i','G_Yaw_d')