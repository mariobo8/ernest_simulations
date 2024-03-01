clearvars; clc; close all;
% data load
normal_un = load("n_force.txt");
force_un = load("fr_force.txt");
wheel_speed = load("fr_vel.txt");
N = 0.001;
force_un = force_un(1:length(wheel_speed));
input_wheel_vel = linspace(0,length(wheel_speed)*N,length(wheel_speed))';

% data def
win = 30;
radius = 0.14; %[m]
st_angle = 1; %[rad]
speed_un = wheel_speed * radius;
speed = movmean(speed_un,win);
input_vel = input_wheel_vel .* radius;
force = movmean(force_un, win);
normal = movmean(normal_un, win);
v_ideal = input_vel ./ cos(st_angle); 

alpha = atan((- input_vel ./ speed + cos(st_angle)) ./ sin(st_angle));

[sorted_alpha, indices] = sort(alpha);

sorted_force = force(indices);
sorted_force = movmean(sorted_force, 1)
figure
plot(sorted_alpha, sorted_force)
%% 
figure
plot(speed);
%% plotting
figure
plot(alpha, force)