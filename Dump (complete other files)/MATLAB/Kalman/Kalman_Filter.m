clc;
clear all;
close all;

%% Constants for the Kalman filter
%model=('Kalman_State_space');

T_s = 5e-3;                 %Sampling time for 9DOF
IPitch = 0.1274;
IRoll  = 0.1274;

% State space representation of system in continues time

% X state space values for A and B
A_c = [0 1 ; 0 0];
B_c = [0; 1/IPitch];
% Y state space values for C and D 
C_c = diag([1,1]);
D_c = 0;


sys = ss(A_c,B_c,C_c,D_c);
sys = c2d(sys,T_s,'ZoH');
[A_d,B_d,C_d,D_d] = ssdata(sys);

Q = eye(2)*10;               %Process noise zero order mean (Not correct values)
R = eye(2)*6.8;             %Measurement noise zero order mean(Not correct values)
P = eye(2)*1000;
K = eye(2);
zhat = [0 0 1]';            %the error made in prediction
gamma = diag([1 1]);      %Used to define the coupling of the process noise..
                            %into the states 
 
%NOTE FOR THE FOLLOWING Q = v*v' AND R = e*e'
%v   = diag([3.16,3.16]);        %Process noise zero order mean (Not correct values)
%e   = diag([2.61 ,2.61]);        %Measurement noise zero order mean(Not correct values)


%K   = ones(2);              %Kalman gain matrix



%P   = diag([2000,2000,2000]);  
I = eye(2);
%F = (I - K*C_d)*A_d;
%H = (I - K*C_d)*B_d;

%sim(model)



dt = 0.06;
duration = 10;

for i=0:dt:duration  
    s = P+R;
    K = A_d*P*inv(s);
    P = A_d *P *A_d' + Q - A_d* P *inv(s) * A_d' * P;
end
% Q = diag ([0 0.01 0.01]);
% gamma = diag ([0 1 1]);
% A_d = [1 0.01 0 ; 0 .99 0.0097; 0 0 0.9512]; 
% B_d = [0.0 0.0002 0.0488];
% C_d = [1 0 0];
% R=0.0025;
% I = eye(3);
p = eye(2)*1000;
for i=0:dt:duration 
    P_star = A_d*p*A_d'+gamma*Q*gamma';  
    K_1 = P_star*C_d'*inv(C_d*P_star*C_d' +R);
    p = (I - K_1*C_d)*P_star;
end




K
K_1
t = 0:dt:duration;
