clc;
clear all;
close all;


%INITIAL GYRO SPPED
wr =0;       %angular rate for roll
wp = 0.001;  %angular rate for pitch
wy = 0;      %angular rate for yaw
w =0;
%KALMAN FILTER TRANSITION MATRIX
S = [0 wy -wp; -wy 0 wr; wp -wr 0]; % matrix used to relate angle to rate of change of angle
A = expm(S); 

%INITIAL VECTORS
x =[0 0 1]';
zhat = [0 0 1]'; %the error made in prediction

%DECLARING ARRAYS TO ENABLE PLOTTING OF DATA
pitch =[];
meas = [];
actual =[];

% SETTING COVARIANCE MATRICES
R = eye(3)*6.8;
Q = eye(3)*10;
P = eye(3)*1000;

%SETTING TIME AND LENGTH OF SIMULATION
dt = 0.06;
duration = 8;

for i=0:dt:duration
    %UPDATING KALMAN FILTER MODEL
    %note A never changes
    x = A*x
    
    %ACCELEROMETER MODEL
    y = x + [0.015+0.01*randn 0.015+0.01*rand 0.015+0.01*randn]' + [(wp-w)/dt 0 0]';
    
    %STORING DATA FOR PLOTTING
    meas = [meas, asin(y(1))];
    
    %SWITCHING MECHANISM 
    if(abs(norm(y)-1)<0.3)
        sigma =1;
    else sigma =0;
    end
    
    Inn = y-zhat;
    s = P+R;
    %GAIN MATRIX
    K = sigma * A*P*inv(s);
    
    %STATE ESTIMATE
    zhat = A * zhat + sigma*K*Inn;
    
    %CONVARIANCE OF PREDICTION ERROR
    P = A *P *A' + Q - sigma * A * P *inv(s) * A' * P;
    
    %UPDATING RATE
    w=wp;
    wp = 0.01*i;
    S = [0 wy -wp; -wy 0 wr; wp -wr 0];
    
    %CALCULATING ESTIMATED ORIENTATION
    
    est_pitch = -asin(zhat(1));
    pitch = [pitch,est_pitch];
    actual = [actual, -asin(x(1))];
    
end

%PLOT DATA
t = 0:dt:duration;
plot(t,pitch,'r',t,meas,'b',t,actual,'g')
    
    
    
        
        
    
    
    
    
    


