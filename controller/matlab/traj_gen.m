clearvars;
close all;
clc;
% Define parameters for the S-shaped path
amplitude = 20;         % Amplitude of the S-shape
frequency = 0.01;        % Frequency of the S-shape
time_interval = 30;     % Total time interval for the path
dt = 0.01;            % Time step
L = 0.8;

% Generate the S-shaped path
t = linspace(0, time_interval, time_interval/dt + 1);
time_vel = t(1:end-1);
x = amplitude * sin(2 * pi * frequency * t);
y = amplitude * sin(4 * pi * frequency * t);

% Calculate velocities
x_dot = diff(x) / dt;
y_dot = diff(y) / dt;

% Calculate heading
psi = atan2(y_dot, x_dot);

% Wrap angles to the range of -2*pi to 2*pi
psi = unwrap(psi);

% Add the first element to the beginning of the vector
firstNumber = psi(1);
psi = [firstNumber, psi];

% Calculate psi_dot
psi_dot = diff(psi) / dt;

%% Calculate additional vectors v and delta
v = x_dot ./ cos(psi(1:end-1));
st_ang = medfilt1(atan(L * psi_dot ./ v),10);
v_j = (x_dot - y_dot)./(cos(psi(1:end-1)) - sin(psi(1:end-1)));
v = medfilt1((y_dot ./ sin(psi(1:end-1))),50);

%%
% Save inputs in a matrix
input = [time_vel', v', st_ang'];

% Save the path, velocities, and heading to a file
pos = [t', x', y', psi'];
pos = pos(:, 1:end-1);

vel = [time_vel', x_dot', y_dot', psi_dot'];
vel = vel(:, 1:end-1);

% Transpose matrices
pos = pos';
vel = vel';
figure
plot( t, x, '-r', t, y, '-b', t, psi, 'g', LineWidth=1.5 )
title('Position')
xlabel('time')
ylabel('value')
legend('x', 'y', 'psi')
grid on

figure
plot( time_vel, x_dot, '-r', time_vel, y_dot, '-b', time_vel, psi_dot, 'g', LineWidth=1.5 )
title('Velocity')
xlabel('time')
ylabel('value')
legend('x_dot', 'y_dot', 'psi_dot')
grid on

figure
plot( time_vel, v, '-r', time_vel, st_ang, '-b', LineWidth=1.5)
title('Control inputs')
xlabel('time')
ylabel('value')
legend('speed', 'steering angle')
grid on

figure
plot(x, y, LineWidth=1.5)
title('Path')
xlabel('X')
ylabel('Y')
grid on


%% save vectors
fileID = fopen('steering_angles.txt', 'w');

% Check if the file was opened successfully
if fileID == -1
    error('Unable to open the file for writing.');
end

% Write the vector to the file
fprintf(fileID, '%f\n', st_ang);

% Close the file
fclose(fileID);

disp('Vector saved to steering_angles.txt');