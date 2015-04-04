%run prop code in folder above. Magnetometer data is stored in SD card. 9dof should be rotated through full sphere.
%the data will have an fragmented last line - this should be deleted. 
%then pass the data through this code - "load data.txt"

%the code plots the calibration elipse - the elipse ideallyy is a sphere centred on (0, 0, 0)
%the code outputs the biases to be subtracted from the register outsputs, and the sensitivity of the scalings
%generally only z needs to be scaled




% %data is an n*9 array, with colums holding acc_x, accy.. gyr_x...._mag_y, mag_z
% 
% load 1_flat.txt
% plot(-X1_flat(:, 7), X1_flat(:, 8))  %plots x vs y - compnesating for magnetometer being sideways
% hold on
% 
% plot(0, 0, 'ro');
% load 4_full.txt
% 
% 
% figure
% 
% 
% scatter3(X4_full(:, 8), -X4_full(:, 7), X4_full(:, 9))
% hold on
% scatter3(0, 0, 0, 'r')
% 
% [centre, radii, ~, ~] = ellipsoid_fit([X4_full(:, 8), -X4_full(:, 7), X4_full(:, 9)], 2);
% 
% disp(centre)
% disp(radii)
% 
% scatter3(centre(1), centre(2), centre(3), 'r')
% 




%data = load('7_new_9dof.txt');
data = load('8_new_dof_calibrated.txt');


%data = load('5_quad_no_motors.txt');
%data = load('mag_data.txt');

figure


scatter3(data(:, 7), data(:, 8), data(:, 9))
hold on
scatter3(0, 0, 0, 'r')

[centre, radii, ~, ~] = ellipsoid_fit([data(:, 7), data(:, 8), data(:, 9)], 2);

disp(centre)
disp(radii)

scatter3(centre(1), centre(2), centre(3), 'r')

xlabel('x')
ylabel('y')
zlabel('z')











