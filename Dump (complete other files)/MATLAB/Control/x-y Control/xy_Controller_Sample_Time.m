%% First Way using PID Tool
s=tf('s');

sys_x = g/s^2;
sys_x_d=c2d(sys_x,Ts,'zoh');
pidtool(sys_x,'PID');
pause
Kp_Pitch = get(C,'Kp');
Ki_Pitch = get(C,'Ki');
Kd_Pitch = get(C,'Kd');
%[A,B,C,D] = tf2ss(g,[1 0 0])
A = [0,1;0,0];
B = [0;9.81];
C = [1,0];
D = 0 ;
states = {'x' 'x_dot'};
inputs = {'D'};
outputs = {'x'};
sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
sys_d = c2d(sys_ss,Ts,'zoh');

A_x_d = sys_d.a;
B_x_d = sys_d.b;
C_x_d = sys_d.c;
D_x_d = sys_d.d;


P=[0.8 0.9];
G_x_d = place(A_x_d',C_x_d',P)';


Kp_Roll= Kp_Pitch;
Ki_Roll= Ki_Pitch;
Kd_Roll = Kd_Pitch;











sys_y = -g/s^2;
sys_y=c2d(sys_y,Tsettling,'zoh');
pidtool(sys_y,'PD')
pause
Kp_Roll= get(C1,'Kp');
Kd_Roll = get(C1,'Kd');
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Saving Controller Parameters
% save('xy_Controller_PD_D_Data','Kp_Roll','Ki_Roll','Kd_Roll','Kp_Pitch','Ki_Pitch','Kd_Pitch','A_x_d','B_x_d','C_x_d','G_x_d')