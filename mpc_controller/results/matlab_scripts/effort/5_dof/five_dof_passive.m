%this code is not ok
clearvars; clc; close all;

%% loading
input_path = fullfile(pwd, '../../../5_dof/passive/input.txt'); % Paths Folder
load(input_path)
state_path = fullfile(pwd, '../../../5_dof/passive/state.txt'); % Paths Folder
load(state_path)
prediction_path = fullfile(pwd, '../../../5_dof/passive/prediction.txt'); % Paths Folder
pred = load(prediction_path);
reference_path = fullfile(pwd, '../../../5_dof/passive/reference.txt'); % Paths Folder
ref = load(reference_path);
path_path = fullfile(pwd, '../../../../path/new_path.txt'); % Paths Folder
path = load(path_path);

load(fullfile(pwd, '../../../5_dof/passive/time_step.txt'))
load(fullfile(pwd, '../../../5_dof/passive/steer_effort.txt'))
load(fullfile(pwd, '../../../5_dof/passive/wheel_effort.txt'))
%% data
N = 10;
dt = 0.1;
lf = 0.16; lr = 0.71;
time = linspace(0,dt*length(input(:,1)),length(input(:,1)))';
coeff = 1;
v_fl = movmean(input(:,1),coeff);
v_fr = movmean(input(:,2),coeff);
v_rl = movmean(input(:,3),coeff);
v_rr = movmean(input(:,4),coeff);
delta_fl = movmean(input(:,5),coeff);
delta_fr = movmean(input(:,6),coeff);
delta_rl = movmean(input(:,7),coeff);
delta_rr = movmean(input(:,8),coeff);
alpha = movmean(input(:,9),coeff);
%beta = atan2((lf .* tan(delta_r) + lr .* (tan(delta_f).*cos(alpha) + sin(alpha))), ...
%     (lf + lr.*(cos(alpha) - tan(delta_f).*sin(alpha))));
virtual_v = movmean(input(:,10),coeff);
%state
x = state(:,1);
y = state(:,2);
psi = state(:,3);

%% path
%TO DO: load the path
x_p = path(:,1);
y_p = path(:,2);

figure
plot(x_p, y_p, 'LineWidth',1.5, 'Color',[0.5 0.5 0.5])
hold on
plot(x,y,'k' ,'LineWidth',1.5); hold on 

grid on
title('trajectory')
xlabel('X [m]')
ylabel('Y [m]')
legend('state', 'reference', 'Location','northeast')

% %% Input
% line_width = 1.5;
% figure
% subplot(321)
% plot(time, v_f, "k", "LineWidth", line_width); 
% grid on; xlim([0, time(end)]); ylim([-0.2, 1]);
% xlabel('time (s)'); ylabel('v_f (m/s)')
% subplot(322)
% plot(time, v_r, "k", "LineWidth", line_width); 
% grid on; xlim([0, time(end)]); ylim([-0.2, 1]);
% xlabel('time (s)'); ylabel('v_r (m/s)')
% subplot(323)
% plot(time, delta_f, "k", "LineWidth", line_width); 
% grid on; xlim([0, time(end)]); ylim([-1.1,1.1]);
% xlabel('time (s)'); ylabel('\delta_f (rad)')
% subplot(324)
% plot(time, delta_r, "k", "LineWidth", line_width);
% grid on; xlim([0, time(end)]); ylim([-1.1,1.1])
% xlabel('time (s)'); ylabel('\delta_r (rad)')
% subplot(325)
% plot(time, alpha, "k", "LineWidth", line_width); 
% grid on; xlim([0, time(end)]); ylim([-1.1,1.1])
% xlabel('time (s)'); ylabel('\alpha (rad)')
% subplot(326)
% plot(time, virtual_v, "k", "LineWidth", line_width); 
% grid on; xlim([0, time(end)]); ylim([-.2,1])
% xlabel('time (s)'); ylabel('v_{virtual} (m/s)')

%% input complete
line_width = 1.5;
%velocities

figure
subplot(211)
plot(time, v_fl, "b", "LineWidth", line_width); 
grid on; xlim([0, time(end)]); ylim([-0.2, 1.2]);
xlabel('time (s)'); ylabel('v_{f} (m/s)')
hold on
plot(time, v_fr, "r", "LineWidth", line_width); 
grid on; xlim([0, time(end)]); ylim([-0.2, 1.2]);
xlabel('time (s)'); ylabel('v_{f} (m/s)')
legend('v_{fl}', 'v_{fr}', Orientation='horizontal')
subplot(212)
plot(time, v_fl, "b", "LineWidth", line_width); 
grid on; xlim([0, time(end)]); ylim([-0.2, 1.2]);
xlabel('time (s)'); ylabel('v_{r} (m/s)')
hold on
plot(time, v_fr, "r", "LineWidth", line_width); 
grid on; xlim([0, time(end)]); ylim([-0.2, 1.2]);
xlabel('time (s)'); ylabel('v_{r} (m/s)')
legend('v_{fl}', 'v_{fr}', Orientation='horizontal')

% subplot(313)
% plot(time, virtual_v, "b", "LineWidth", line_width); 
% grid on; xlim([0, time(end)]); ylim([-.2,1])
% xlabel('time (s)'); ylabel('v_{virtual} (m/s)')

%steering 
figure
subplot(311)
plot(time, delta_fl, "r", "LineWidth", line_width); 
grid on; xlim([0, time(end)]); ylim([-1.5,1.5]);
xlabel('time (s)'); ylabel('\delta_{f} (rad)')
hold on
plot(time, delta_fr, "b", "LineWidth", line_width);
grid on; xlim([0, time(end)]); ylim([-1.5,1.5])
xlabel('time (s)'); ylabel('\delta_{fr} (rad)')
legend('\delta_{fl}', '\delta_{fr}', Orientation='horizontal')
subplot(312)
plot(time, delta_rl, "r", "LineWidth", line_width); 
grid on; xlim([0, time(end)]); ylim([-1.5,1.5]);
xlabel('time (s)'); ylabel('\delta_{r} (rad)')
hold on
plot(time, delta_rr, "b", "LineWidth", line_width);
grid on; xlim([0, time(end)]); ylim([-1.5,1.5])
xlabel('time (s)'); ylabel('\delta_{r} (rad)')
legend('\delta_{rl}', '\delta_{rr}', Orientation='horizontal')
subplot(313)
plot(time, alpha, "b", "LineWidth", line_width); 
grid on; xlim([0, time(end)]); ylim([-1.1,1.1])
xlabel('time (s)'); ylabel('\alpha (rad)')

% %% skidding eval
% %beta
% 
% skid = v_f .* cos(alpha+delta_f) - v_r .* cos(delta_r);
% figure
% plot(time,skid, "k", "LineWidth",line_width)
% title('Skidding Factor')
% xlabel('time (s)')
% ylabel('Skidding (m/s)')
% xlim([0 time(end)]); ylim([-0.2 0.2])
% grid on


%% time step

figure
time_step_ms = time_step .* 1000;
avg_step = mean(time_step_ms);
plot(time_step_ms,"k" ,"LineWidth",line_width)
hold on
yline(avg_step,"-r","LineWidth",line_width)
grid on 
xlim([0 length(time)]); ylim([0 150])
xlabel('MPC Step'); ylabel('Solving Time (ms)')


%% STEERING TORQUE
%NAME torque
%TODO: change labels add filter and limits
f_torque = time(end)/length(steer_effort(:,1));
t_torque_m = 0:f_torque:time(end);
t_torque = t_torque_m(1:end-1);

line_width = 1;
figure
sgtitle('Steering Torque');
subplot(321)
plot(t_torque, steer_effort(:,1), "k", "LineWidth", line_width); 
grid on; xlim([0, time(end)]);
xlabel('time (s)'); ylabel('T_{fl} (Nm)')
subplot(322)
plot(t_torque, steer_effort(:,2), "k", "LineWidth", line_width); 
grid on; xlim([0, time(end)]); 
xlabel('time (s)'); ylabel('T_{fr} (Nm)')
subplot(323)
plot(t_torque, steer_effort(:,3), "k", "LineWidth", line_width); 
grid on; xlim([0, time(end)]);
xlabel('time (s)'); ylabel('T_{rl} (Nm)')
subplot(324)
plot(t_torque, steer_effort(:,4), "k", "LineWidth", line_width);
grid on; xlim([0, time(end)]); 
xlabel('time (s)'); ylabel('T_{rr} (Nm)')
subplot(325)
plot(t_torque, steer_effort(:,5), "k", "LineWidth", line_width); 
grid on; xlim([0, time(end)]); 
xlabel('time (s)'); ylabel('T_{pivot} (Nm)')


%% Wheel TORQUE
%NAME torque
%TODO: change labels add filter and limits
f_torque = time(end)/length(wheel_effort(:,1));
t_torque_m = 0:f_torque:time(end);
t_torque = t_torque_m(1:end-1);

line_width = 1;
figure
sgtitle('Wheel Torque');
subplot(221)
plot(t_torque, wheel_effort(:,1), "k", "LineWidth", line_width); 
grid on; xlim([0, time(end)]);
xlabel('time (s)'); ylabel('T_{fl} (Nm)')
subplot(222)
plot(t_torque, wheel_effort(:,2), "k", "LineWidth", line_width); 
grid on; xlim([0, time(end)]); 
xlabel('time (s)'); ylabel('T_{fr} (Nm)')
subplot(223)
plot(t_torque, wheel_effort(:,3), "k", "LineWidth", line_width); 
grid on; xlim([0, time(end)]);
xlabel('time (s)'); ylabel('T_{rl} (Nm)')
subplot(224)
plot(t_torque, wheel_effort(:,4), "k", "LineWidth", line_width);
grid on; xlim([0, time(end)]); 
xlabel('time (s)'); ylabel('T_{rr} (Nm)')

%% energy computation 
%steer
tk = 0.05; %Nm/A torque constant
V = 80; %V
st_eff_new = interp1(t_torque', steer_effort, time);
nan_indices = isnan(st_eff_new);
st_eff_new(nan_indices) = 0;
I = st_eff_new ./ tk; %current
P_st = abs(V.*I.*1e-3); %power KW
E_st = trapz(time, P_st) / 3600
E_t_st = sum(E_st)
figure
for i = 1 : size(P_st,2)
    plot(time, P_st(:,i))
    hold on
end
xlabel('time (s)'); ylabel('Power (W)')
legend('\delta_{fl}','\delta_{fr}','\delta_{rl}','\delta_{rr}','pivot', 'Orientation','horizontal')
grid on
%wheel 
tk = 0.05; %Nm/A torque constant
V = 80;
wheel_eff_new = interp1(t_torque', wheel_effort, time);
nan_indices = isnan(wheel_eff_new);
wheel_eff_new(nan_indices) = 0;
I_w = wheel_eff_new ./ tk; %current
P_w = abs(V*I_w*1e-3); %power KW
E_fl = trapz(time, P_w(:,1)) / 3600;
E_fr = trapz(time, P_w(:,2)) / 3600;
E_rl = trapz(time, P_w(:,3)) / 3600;
E_rr = trapz(time, P_w(:,4)) / 3600;
E_wheel = E_fl + E_fr + E_rl + E_rr

E_tot = E_t_st + E_wheel

%%



% %% trajectory plotting
% 
% figure(500)
% 
% % Animate the robot motion
% %figure;
% set(gcf,'Position',[25 25 920 720]);
% %set(gcf,'PaperPositionMode','auto')
% set(gcf, 'Color', 'w');
% %set(gcf,'Units','normalized','OuterPosition',[10 0 0.55 1]);
%     plot(x_p, y_p,'-k','linewidth',1);
%     hold on
% line_width = 1.5;
% fontsize_labels = 15;
% pred = pred(2:end,:);
% ref = ref(2:end,:);
% x_r_1 = [];
% y_r_1 = [];
% 
% for k = 1:size(pred,1)
%     h_t = 0.25; w_t=0.12; % triangle parameters
%     %plot path
%     plot(x_p, y_p,'-k','linewidth',1);
%     hold on;
%     if (1+(k-1)*(N+1)) > size(pred,1)
%         break
%     end
%     x1 = pred(1+(k-1)*(N+1),1); y1 = pred(1+(k-1)*(N+1),2); th1 = pred(1+(k-1)*(N+1),3); %state
%     x_r_1 = [x_r_1 x1];
%     y_r_1 = [y_r_1 y1];
% 
%     x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
%     y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];
% 
%     plot(x_r_1,y_r_1,'-r','linewidth',line_width);hold on % plot exhibited trajectory
%     if k < size(pred,1) % plot prediction
%         plot(pred(1+(k-1)*(N+1):(N+1)*k,1),pred(1+(k-1)*(N+1):k*(N+1),2),'r--*') %pred
%         hold on
%         %plot(ref(1+(k-1)*(N-1):k*(N-1),1),ref(1+(k-1)*(N-1):(N-1)*k,2), 'g--*') %ref
%         plot(ref(k,1),ref(k,2), 'g--*') %ref
%     end
% 
%     hold on
%     fill(x1_tri, y1_tri, 'blue'); % plot robot position
%     hold off
%     ylabel('$y$ (m)','interpreter','latex','FontSize',fontsize_labels)
%     xlabel('$x$ (m)','interpreter','latex','FontSize',fontsize_labels)
%     legend('Path','Executed','Predicted', 'reference')
%     axis([-12  0.8 -1 5.5]) 
%     pause(0.001)
%     box on;
%     grid on
%     %aviobj = addframe(aviobj,gcf);
%     drawnow
%     F(k) = getframe(gcf); % to get the current frame
% end
% close(gcf)
% video = VideoWriter('pfws_flat_path.avi','Uncompressed AVI');
% video.FrameRate = 10;  % (frames per second) 
% open(video)
% writeVideo(video,F)
% close (video)
% 
% % % scatter(pred(N:N*2,1), pred(N:N*2,2))