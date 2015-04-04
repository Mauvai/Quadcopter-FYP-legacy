
if 0
    dof1 = read_9dof_file(1, '9dof');
    dof2 = read_9dof_file(2, '9dof');
    dof3 = read_9dof_file(3, '9dof');
    dof4 = read_9dof_file(4, '9dof');
end
if 0
    disc1 = read_9dof_file(1, 'disc');
    disc2 = read_9dof_file(2, 'disc');
    disc3 = read_9dof_file(3, 'disc');
    disc4 = read_9dof_file(4, 'disc');
    discovery_data = {disc1, disc2, disc3, disc4};
end




% Motor down 2 is 0.675kOhm, motor 3 down is 3.00 kOhm (pot)   
% Sense res is 3.3kOhm.    5V supply voltage. Measured voltages are across pots

%must frist generate calbiration curve

R1 = 3300;   %sense resistance
Vss = 5;

centre = 61.8; %refrence position, quadcopter is level. measurements in cm
centreV = 1.9; %voltage at centre pos

position = [12.8, 25.7, 43,  74.2, 87.6, 97.4];
voltage  = [1,    1.26, 1.6, 2.13, 2.33, 2.42;];


%R2 is the pot resistance, and is variable
R2 = (voltage.*R1)./(Vss-voltage);

angle = atan((centre-position)./(82.5));
angle = angle.*(180/pi);

if 0
   % plot to demonstrate that the pot resistance varies linearly with angle
   plot(angle, R2) 
   title('Pot resistance Vs. angle (demonstrates pot is linear with anlgle)');
end


p = polyfit(R2, angle, 1);   % fit a line to the data, such that inputting a 
                             % resistance as x gives out an angle
figure
hold on
title('Pot angle Vs. time. blue is Test 1, green T2, red T3 and cyan T4')
colour = ['b', 'g', 'r', 'c'];
for i = 1:4
    Rmat = (discovery_data{i}{2}.*R1)./(Vss - discovery_data{i}{2});
    theta = polyval(p, Rmat);
    plot(discovery_data{i}{1}, theta, colour(i))
end

Rmat = (discovery_data{i}{2}.*R1)./(Vss - discovery_data{i}{2});
    theta = polyval(p, Rmat);
    plot(discovery_data{i}{1}, theta, colour(i))























