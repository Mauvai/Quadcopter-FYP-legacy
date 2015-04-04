%% Measured Physical Parameters
% These measurements were taken on the quadcopter

clear all
close all
clc

%% Plot Characteritics
LineThickness = 1.5;
%% Constants
g=9.81;
%% Mass and size of quadcopter
% Mass of quadcopter taken using digital scales and added 2 Arduino boards
% to approximate the electronics yet to be added
Mass=2.26;
L = 0.348;                      %Distance between motor and centre of gravity

%% Pulse Lengths in milliseconds
PWM = [1.1 1.15 1.2 1.25 1.3 1.35 1.4 1.45 1.5 1.55 1.6 1.65 1.7 1.75 1.8 1.85 1.9 1.95]% 2];

%% Motor Speeds in rad/s
Motor1_Speed = [145.0368608	198.9675347	242.9498319	280.6489437	310.4940739	337.7212103	362.3303527	384.8451001	405.7890511	429.350996	456.0545335	483.2816699	506.320016	521.5043805	544.5427266	561.2978874	573.8642581	589.5722213];%	594.2846103];
Motor2_Speed = [161.7920217 215.7226955 258.6577951 295.8333082 326.2020372 356.0471674 377.7765166 401.0766621 423.0678107 451.3421446	481.7108736	503.1784233	531.4527572	551.8731095	570.7226654	586.9542274	601.6149932	614.1813638];%	611.5633699];
Motor3_Speed = [152.8908425	206.8215164	248.1858196	287.9793266	318.8716543	345.0515931	367.5663405	391.6518841	409.9778413	432.4925886	458.6725274	483.8052687	508.9380099	530.4055597	549.2551156	566.5338752	581.1946409	590.6194189];%	591.6666164];
Motor4_Speed = [157.6032315	210.2249084	256.0398013	290.5973205	318.3480556	346.0987907	370.7079331	393.2226805	415.9992272	444.5353605	472.2860956	496.895238	520.9807817	538.2595413	553.4439058	572.2934617	585.3834311	596.9026042];%	594.2846103];
figure
Motor_Speeds =[Motor1_Speed;Motor2_Speed;Motor3_Speed;Motor4_Speed];
plot(PWM,Motor_Speeds,'LineWidth', LineThickness);
grid on
title('Motor Speed')
xlabel('Length of Pulse [milliseconds]')
ylabel('Motor Speed [rpm/s]')
hold on
% Fit Trenldine
Motor_Speed_Data=[Motor1_Speed ,Motor2_Speed, Motor3_Speed, Motor4_Speed];
PWM_Speed=[PWM, PWM, PWM ,PWM];
coeff_speed=polyfit(PWM_Speed, Motor_Speed_Data,1);
xFitting=1:2;
yFitted=polyval(coeff_speed, xFitting);
plot(xFitting,yFitted,'k','LineWidth', LineThickness);

saveas(gcf, 'MotorSpeed.png')
%% Motor Thrusts in Newtons
Motor1_Thrust = [0.3924	0.73575	1.0791	1.5696	1.91295	2.2563	2.69775	3.09015	3.4335	3.924	4.36545	5.05215	5.64075	6.1803	6.6708	7.1613	7.6518	8.1423];
Motor2_Thrust = [0.3924	0.6867	1.03005	1.3734	1.7658	2.10915	2.50155	2.89395	3.28635	3.8259	4.4145	4.905	5.44455	6.03315	6.52365	7.01415	7.60275	7.99515];
Motor3_Thrust = [0.2943	0.5886	0.93195	1.52055	1.8639	2.3544	2.69775	3.1392	3.5316	3.924	4.46355	5.0031	5.6898	6.2784	6.7689	7.30845	7.6518	7.99515];
Motor4_Thrust = [0.34335	0.73575	1.12815	1.52055	1.91295	2.30535	2.7468	3.09015	3.6297	4.2183	4.8069	5.3955	6.0822	6.71985	7.1613	7.70085	8.1423	8.58375];

Motor_Thrusts = [Motor1_Thrust;Motor2_Thrust;Motor3_Thrust;Motor4_Thrust];
figure
plot(PWM(1:18),Motor_Thrusts,'LineWidth', LineThickness);
grid on
title('Motor Thrust')
xlabel('Length of Pulse [milliseconds]')
ylabel('Motor Thrusts [Newtons]')
hold on

%Fit Trendline
Motor_Thrust_Data=[Motor1_Thrust, Motor2_Thrust, Motor3_Thrust, Motor4_Thrust];
PWM_Thrust=[PWM(1:18), PWM(1:18), PWM(1:18) ,PWM(1:18)];
coeff_thrust=polyfit(PWM_Thrust, Motor_Thrust_Data,1);
xFitting=1:2;
yFitted=polyval(coeff_thrust, xFitting);
plot(xFitting,yFitted,'k','LineWidth', LineThickness);
saveas(gcf, 'MotorThrust.png')

Kf = coeff_thrust(1);
F_yIntercept = coeff_thrust(2);
%% Inertia
%Inertia test completed with rod rotating around a metal metal rod
%connected by wire. 

%% Yaw Inertia
%Damping was presumed negligble based on observation
TYawTime = [18.08, 18.18 18.14,18.08, 18.18,18.36];
TYawNumber = [10,10, 10,10,10,10];
TYawT = TYawTime./TYawNumber;
TYaw =sum(TYawT)/length(TYawT);                     %Period of Yaw Oscillation
LengthYaw = 0.662;%0.82;                              %Dist from Rod to Centre of Mass
wYaw=2*pi/TYaw;                                     %Natural Frequency of Yaw Axis
IYawParallel= Mass*g*LengthYaw*(TYaw/(2*pi))^2;     %Before parallel axis accounted for

IYaw=IYawParallel-(Mass*(LengthYaw^2));

%% Pitch/Roll Inertia
%Symmetry was assumed between the pitch and roll axes.
%Damping was having a discernible effect and needed to be measured. Period
%and distance between rod and Centre of Mass were taken and used to
%calculate then inertia. The parallel axes theorem was used to calculate
%inertia of the quadcopter.

% Pitch - Motors 1 & 4, 1 on top
TPitchTime =[ 17.32, 17.36, 17.37, 17.22, 17.51, 17.14 ];             %Time for 
TPitchNumber = [ 10, 10, 10, 10, 10, 10 ];                  %Number of oscillations
TPitchT=TPitchTime./TPitchNumber;           %Period

TPitch=sum(TPitchT)/length(TPitchT);        %Average Period of oscillation
LengthPitch = 0.66;

IPitchParallel= Mass*g*LengthPitch*(TPitch/(2*pi))^2; %Before parallel axis accounted for

IPitch=IPitchParallel-(Mass*(LengthPitch^2));     %Inertia of the Pitch and Roll axes

% Roll - Motors 2 & 3, 3 on top
TRollTime =[17.12, 17.22, 17.36, 17.21 ];             %Time for 
TRollNumber = [10, 10, 10, 10 ];                  %Number of oscillations
TRollT=TRollTime./TRollNumber;           %Period

TRoll=sum(TRollT)/length(TRollT);        %Average Period of oscillation
LengthRoll = 0.655;

IRollParallel= Mass*g*LengthRoll*(TRoll/(2*pi))^2; %Before parallel axis accounted for

IRoll=IRollParallel-(Mass*(LengthRoll^2));     %Inertia of the Pitch and Roll axes
%% Damping of Pitch/Roll Axes
%Laser pointer was used to mark max amplitude of oscillation on paper on
%ground. Using atan and distance between Centre of Rotation osccilation
%angle was found. Oscillation gave a damped exponential. The natural log of
%this was then graph and a trend line fitted. The coefficients of this line
%gave gamma. From the natural frequency could be found and hence the
%damping ratio

% Ross Data
Dist1 = atan([350 302 288 268 242 204 182 166 148 131 119 102 88 79 65 56 49 44 41 39 36 34 ]/1705);
DistTime1=1.7456*[0:length(Dist1)-1];
figure(6)
plot(DistTime1,Dist1,'o','LineWidth', LineThickness);
grid on
title('Pendulum Test')
xlabel('Time [Seconds]')
ylabel('Angle [Degrees]')
hold on
%saveas(gcf, 'PendulumAmplitude.png')
% Simon Data 1
Dist2 = atan([276 258 236 220 214 188 164 146 134 119 103 88 76 65 56 49 44 41 36]/1705);
DistTime2 = (1.7456*[0:length(Dist2)-1])+(1.7456/2);
plot(DistTime2,Dist2,'or','LineWidth', LineThickness);
% Simon Data 2
Dist3 = atan([300 276 253 230 211 195 172 160 142 126 110 96 82 69 57 52 47 42 36 34 31 28]/1705);
DistTime3 = (1.7456*[0:length(Dist3)-1])+(1.7456/2);
plot(DistTime3,Dist3,'om','LineWidth', LineThickness);
% Simon Data 3
Dist4 = atan([293 271 250 227 205 186 165 149 132 113 98 82 70 58 49 42 38 33 31 28]/1705);
DistTime4 = (1.7456*[0:length(Dist4)-1])+(1.7456/2);
plot(DistTime4,Dist4,'og','LineWidth', LineThickness);



%Plotting all data on a log scale
DistConcat=log([Dist1,Dist2,Dist3,Dist4]);
DistTimeConcat=[DistTime1, DistTime2, DistTime3, DistTime4];

figure
plot(DistTimeConcat, DistConcat,'o','LineWidth', LineThickness)
hold on
grid on
%title('Natural log of Quad Pendulum Amplitudess')
xlabel('Time [seconds]')
ylabel('Angle of Deflection [Degrees]')
%saveas(gcf, 'PendulumLog.png')

% Find and plot a linear trendline
coeff=polyfit(DistTimeConcat, (DistConcat),1);
xFitting=0:40;
yFitted=polyval(coeff, xFitting);
plot(xFitting,yFitted,'r','LineWidth', LineThickness);

%Fit Envelope to Damped Exponential
figure(6)
plot(DistTime4,exp((coeff(1)*DistTime4)+coeff(2)),'LineWidth', LineThickness)
%saveas(gcf, 'PendulumAmplitude.png')


% Calculate the missing variables
gamma=-coeff(1);
wd=2*pi/TPitch;
wn=sqrt(wd^2+gamma^2);
zeta=gamma/wn;
k= Mass*wn;
B_Pitch= 2*zeta*sqrt(wn*k);


%% Parameters Found
I_Psi =IYaw;
I_Gamma = IRoll;
I_Theta=IPitch;
Kf;
L;
B_Pitch;                  % Damping
F_yIntercept;
%% Parameters Not Found
Motor_TimeConstant = 70e-3;
J = 80e-9;
Km=0;
Kw=0;
w0_yaw=0;
D=0;

%% Save Relevant Parameters
save('Parameters','I_Psi', 'I_Gamma', 'I_Theta','Kf','L','B_Pitch','F_yIntercept', 'Motor_TimeConstant','J','Km','Kw','w0_yaw','D')
