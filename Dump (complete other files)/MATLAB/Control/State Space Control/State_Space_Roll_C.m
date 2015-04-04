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
%% Model Derivation
A = [0, 1, 0
     0, 0, 2*Kf*L/(I_Theta)
     0, 0, -1/(Motor_TimeConstant)];
 
 B = [0
      0
      1/Motor_TimeConstant];
  
  C= [1 0 0
      0 1 0];
  
  D=[0;
    0];

states = {'theta' 'theta_dot' 'omega'};
inputs = {'D'};
outputs = {'theta'; 'theta_dot'};
sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
sys_d = c2d(sys_ss,Ts,'zoh');

%% Controllability and Observability
co = ctrb(sys_d);
ob = obsv(sys_d);

controllability = rank(co);
observability = rank(ob);
 

%% Pole Placement Design


zeta = 1/sqrt(2);
w_n = 1;

p1 = -zeta*w_n + sqrt(zeta*w_n^2-w_n^2);
p2 = -zeta*w_n - sqrt(zeta*w_n^2-w_n^2);
p3 = -14;

K_Roll_C = place(A,B,[p1 p2 p3]);
Nbar=1/(C*(B*K_Roll_C-A)^-1*B);
Nbar_Roll_C=Nbar(1);
sys_cl = ss(A-B*K_Roll_C,Nbar_Roll_C*B,C,0);

%% Step Response
% t = 0:0.01:2;
% [y,t,x]=step(sys_cl,t);%lsim(sys_cl,r,t);
% figure
% [AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
% set(H1,'linewidth',linesize)% to change the first line
% set(H2,'linewidth',linesize) % to change the second line
% set(get(AX(1),'Ylabel'),'String','Roll Angular Position [rad]')
% set(get(AX(2),'Ylabel'),'String','Roll Angular Velocity [rad/s]')
% title('Step Response')
% saveas(gcf, 'Roll_Step_LSIM.png')

%% Simulink
model = 'Roll_Diff_SS_C';
sim(model)

figure
subplot(4,1,1)
plot(Theta)
title('Roll Angle')
grid on
subplot(4,1,2)
plot(Theta_dot)
title('Roll Angle Rate')
grid on
subplot(4,1,3)
plot(Thrust)
title('Thrust')
grid on
subplot(4,1,4)
plot(PWM)
title('PWM')
grid on
save('Roll_Controller_SS_C_Data','Nbar_Roll_C','K_Roll_C')