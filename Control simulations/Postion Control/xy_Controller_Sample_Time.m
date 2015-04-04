close all
load('Model_Parameters')
F0=Mass*g;
s=tf('s');
%% Design PI Controller using 'pidtune'
wc=0.2:0.2:0.6;
sys_x= (F0) / (s*(s*Mass+ViscousDamping));
sys_x_d=c2d(sys_x,Ts2,'zoh');
for i=1:length(wc)
C=pidtune(sys_x_d,'pi',wc(i));
T_pi=feedback(sys_x_d*C,1);
step(T_pi)
hold on
xlim([0 150])
end
legend(['BW of ',num2str(wc(1))],['BW of ',num2str(wc(2))],['BW of ',num2str(wc(3))],'Location','Best')
title('Step Response of position under PI control')
ylabel('Position [m]')
grid on
saveas(gcf,'pi_xy_response.png')
C=pidtune(sys_x_d,'pi',0.4);
Kp_x = get(C,'Kp');
Ki_x = get(C,'Ki');
Kd_x = get(C,'Kd');

Kp_y= Kp_x;
Ki_y= Ki_x;
Kd_y = Kd_x;
%% Observer
% Continuous State Space System
A_x = [0,1;0,-ViscousDamping/Mass];
B_x = [0;9.81];
C_x = [1,0];
D_x = 0 ;
states = {'x' 'x_dot'};
inputs = {'D'};
outputs = {'x'};
sys_ss = ss(A_x,B_x,C_x,D_x,'statename',states,'inputname',inputs,'outputname',outputs);
sys_d = c2d(sys_ss,Ts2,'zoh');
% Convert Model to Digital State Space System
A_x_d = sys_d.a;
B_x_d = sys_d.b;
C_x_d = sys_d.c;
D_x_d = sys_d.d;

% Place Poles of Observer
P=[0.8 0.9];
G_x_d = place(A_x_d',C_x_d',P)';
%% Saving Controller Parameters
save('xy_Controller_PD_D_Data','Kp_y','Ki_y','Kd_y','Kp_x','Ki_x','Kd_x','A_x_d','B_x_d','C_x_d','G_x_d')
disp('Controller Variables Updated')