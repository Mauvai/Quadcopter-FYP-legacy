%% Damping of Pitch/Roll Axes
%Laser pointer was used to mark max amplitude of oscillation on paper on
%ground. Using atan and distance between Centre of Rotation osccilation
%angle was found. Oscillation gave a damped exponential. The natural log of
%this was then graph and a trend line fitted. The coefficients of this line
%gave gamma. From the natural frequency could be found and hence the
%damping ratio

close all 
% Ross Data
Dist1 = atan([350 302 288 268 242 204 182 166 148 131 119 102 88 79 65 56 49 44 41 39 36 34 ]/1705);
DistTime1=1.7456*[0:length(Dist1)-1];
figure(7)
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
DistConcat=([Dist1,Dist2,Dist3,Dist4]);
DistTimeConcat=[DistTime1, DistTime2, DistTime3, DistTime4];

figure(8)
semilogy(DistTimeConcat, DistConcat,'o','LineWidth', LineThickness)
hold on
grid on
%title('Natural log of Quad Pendulum Amplitudess')
xlabel('Time [seconds]')
ylabel('Angle of Deflection [Degrees]')
%saveas(gcf, 'PendulumLog.png')

% Find and plot a linear trendline
coeff=polyfit(DistTimeConcat, log(DistConcat),1);
xFitting=0:40;
yFitted=exp(polyval(coeff, xFitting));
figure(8)
semilogy(xFitting,yFitted,'r','LineWidth', LineThickness);

%Fit Envelope to Damped Exponential
figure(7)
plot(DistTime4,exp((coeff(1)*DistTime4)+coeff(2)),'LineWidth', LineThickness)
%saveas(gcf, 'PendulumAmplitude.png')


% Calculate the missing variables
gamma=-coeff(1);
wd=2*pi/TPitch;
wn=sqrt(wd^2+gamma^2);
zeta=gamma/wn;
k= Mass*wn;
B_Pitch= 2*zeta*sqrt(wn*k);
