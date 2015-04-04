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
Ts = 1/80;
sim_length=30;
omega_0=520;

Drag=D;

YawStep=1;
%% Model Derivation
A = [0, 1, 0
     0, 0, (4*Kw/I_Psi)*( (2*omega_0*D) - J)
     0, 0, -1/Motor_TimeConstant];
 
 B = [0
      4*Kw*J/(Motor_TimeConstant*I_Psi)
      1/Motor_TimeConstant];
  
  C= [1 0 0
      0 1 0];
  
  D=[0;
    0];

states = {'theta' 'theta_dot' 'omega'};
inputs = {'D'};
outputs = {'theta'; 'theta_dot'};
sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
%sys_d = c2d(sys_ss,Ts,'zoh');


%% Controllability and Observability
co = ctrb(sys_ss);
ob = obsv(sys_ss);

controllability = rank(co);
observability = rank(ob);

%% System Poles
poles = eig(A);

%% LQR Design
Q =  [1     0     0
     0     1     0
     0     0     1e-6]
R = 1;
K_Yaw_C = lqr(A,B,Q,R)

Ac = [(A-B*K_Yaw_C)];
Bc = [B];
Cc = [C];
Dc = [D];

states = {'yaw' 'yaw_dot' 'omegad'};
inputs = {'r'};
outputs = {'yaw'; 'yaw_dot'};

sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:5;
r =0.2*ones(size(t));
[y,t,x]=lsim(sys_cl,r,t);
figure
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','Yaw(s)')
set(get(AX(2),'Ylabel'),'String','Yaw Rate(radians/s)')
title('Step Response with LQR Control')


Nbar=1/(C*(B*K_Yaw_C-A)^-1*B);
Nbar1=Nbar(1);

sys_cl = ss(Ac,Bc*Nbar1,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:5;
r =0.2*ones(size(t));
[y,t,x]=lsim(sys_cl,r,t);
figure
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','Yaw(s)')



%% Simulink of Yaw
model='Yaw_Diff_SS_C';
sim(model)

figure
plot(Setpoint,'g')
hold on
plot(Yaw);
%ylim([-10 10])
title('Simulink: Yaw');
ylabel('Yaw [rad]')
xlabel('Time [seconds]')
grid on
saveas(gcf, 'Yaw_Yaw.png')
figure
plot(YawRate);
title('Simulink: Yaw Rate');
ylabel('Yaw Rate [rad/s]');
xlabel('Time [seconds]')
grid on
saveas(gcf, 'Yaw_Yaw.png')
figure
plot(Pulse);
title('PWM');


%% Save Paraemeters 
save('Yaw_Controller_SS_C_Data','K_Yaw_C')