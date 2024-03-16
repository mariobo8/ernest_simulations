clearvars; clc; close all;
%% loading
steer_torque = fullfile(pwd, '../steer_torque.txt'); % Paths Folder
wheel_torque = fullfile(pwd, '../wheel_torque.txt'); 
load(steer_torque)
load(wheel_torque)
time = 0:0.001:.799;

fl_steer = steer_torque(:,1);
fr_steer = steer_torque(:,2);
rl_steer = steer_torque(:,3);
rr_steer = steer_torque(:,4);
pivot = steer_torque(:,5);

fl_wheel = wheel_torque(:,1);
fr_wheel = wheel_torque(:,2);
rl_wheel = wheel_torque(:,3);
rr_wheel = wheel_torque(:,4);
%% plot
figure
plot(fl_steer-mean(fr_steer))
figure
plot(fr_steer)
figure
plot(rl_steer)
figure
plot(rr_steer)
figure
plot(pivot)

%% plot

figure
plot(fl_wheel)
figure
plot(fr_wheel)
figure
plot(rl_wheel)
figure
plot(rr_wheel)

%% 
