%data is an n*9 array, with colums holding acc_x, accy.. gyr_x...._mag_y, mag_z



data = load('9_no motors_2d_flat_calibrated.txt');

plot(data(:, 7), data(:, 8))  %plots x vs y - magnetometer calibration internal
hold on

plot(0, 0, 'ro');
[centre, ~, ~, ~] = fitellipse([data(:, 7), data(:, 8)]);
plot(centre(1), centre(2), 'ro')
disp(centre)
figure
plot(data(:, 9))

figure
data = load('10_motors_2d_calibrated.txt');

plot(data(:, 7), data(:, 8))  %plots x vs y - magnetometer calibration internal
hold on

plot(0, 0, 'ro');
[centre, ~, ~, ~] = fitellipse([data(:, 7), data(:, 8)]);
plot(centre(1), centre(2), 'ro')
disp(centre)
figure
plot(data(:, 9))