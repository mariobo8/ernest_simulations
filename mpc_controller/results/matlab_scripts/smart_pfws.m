clearvars; clc; close all;
%% loading
load(fullfile(pwd, '../4ws_pivot_smart/input.txt'))
load(fullfile(pwd, '../4ws_pivot_smart/time_step.txt'))
load(fullfile(pwd, '../4ws_pivot_smart/wheel_effort.txt'))
load(fullfile(pwd, '../4ws_pivot_smart/state.txt'))
load(fullfile(pwd, '../4ws_pivot_smart/steer_effort.txt'))
load(fullfile(pwd, '../../path/std_path.txt'))
pred_k = load(fullfile(pwd, '../4ws_pivot_smart/prediction.txt'));
ref_k = load (fullfile(pwd, '../4ws_pivot_smart/reference.txt'));
%% data
N = 10;
dt = 0.1;
lf = 0.16; lr = 0.71;
time = linspace(0,dt*length(input(:,1)),length(input(:,1)))';
coeff = 1;
v_f = movmean(input(:,1),coeff);
v_r = movmean(input(:,2),coeff);
delta_f = movmean(input(:,3),coeff);
delta_r = movmean(input(:,4),coeff);
alpha = movmean(input(:,5),coeff);
beta = atan2((lf .* tan(delta_r) + lr .* (tan(delta_f).*cos(alpha) + sin(alpha))), ...
     (lf + lr.*(cos(alpha) - tan(delta_f).*sin(alpha))));
virtual_v = movmean(input(:,6),coeff);
%state
x = state(:,1);
y = state(:,2);
psi = state(:,3);

%% path
%TO DO: load the path
x_p = std_path(:,1);
y_p = std_path(:,2);

figure
plot(x_p, y_p, 'LineWidth',1.5, 'Color',[0.5 0.5 0.5])
hold on
plot(x,y,'k' ,'LineWidth',1.5); hold on 

grid on
title('trajectory')
xlabel('X [m]')
ylabel('Y [m]')
legend('state', 'reference', 'Location','northeast')

%% Input
line_width = 1.5;
figure
subplot(321)
plot(time, v_f, "k", "LineWidth", line_width); 
grid on; xlim([0, time(end)]); ylim([-0.2, 1]);
xlabel('time (s)'); ylabel('v_f (m/s)')
subplot(322)
plot(time, v_r, "k", "LineWidth", line_width); 
grid on; xlim([0, time(end)]); ylim([-0.2, 1]);
xlabel('time (s)'); ylabel('v_r (m/s)')
subplot(323)
plot(time, delta_f, "k", "LineWidth", line_width); 
grid on; xlim([0, time(end)]); ylim([-1.1,1.1]);
xlabel('time (s)'); ylabel('\delta_f (rad)')
subplot(324)
plot(time, delta_r, "k", "LineWidth", line_width);
grid on; xlim([0, time(end)]); ylim([-1.1,1.1])
xlabel('time (s)'); ylabel('\delta_r (rad)')
subplot(325)
plot(time, alpha, "k", "LineWidth", line_width); 
grid on; xlim([0, time(end)]); ylim([-1.1,1.1])
xlabel('time (s)'); ylabel('\alpha (rad)')
subplot(326)
plot(time, virtual_v, "k", "LineWidth", line_width); 
grid on; xlim([0, time(end)]); ylim([-.2,1])
xlabel('time (s)'); ylabel('v_{virtual} (m/s)')

%% rate of change
omega_f = [0; diff(delta_f)];
omega_r = [0; diff(delta_r)];
omega_a = [0; diff(alpha)];


en_coeff = sum((abs(omega_f)*2 + abs(omega_r)*2 ...
    + abs(omega_a)*2)*dt);
figure
subplot(311)
sgtitle(num2str(en_coeff))
plot(time, omega_f,'k', 'LineWidth',line_width)
grid on; xlim([0, time(end)]); ylim([-0.09 0.09])
xlabel('time (s)'); ylabel('\omega_f (m/s)')
subplot(312)
plot(time, omega_r,'k', 'LineWidth',line_width)
grid on; xlim([0, time(end)]); ylim([-0.09 0.09])
xlabel('time (s)'); ylabel('\omega_r (m/s)')
subplot(313)
plot(time, omega_a,'k', 'LineWidth',line_width)
grid on; xlim([0, time(end)]); ylim([-0.09 0.09])
xlabel('time (s)'); ylabel('\alpha (m/s)')

disp(en_coeff)



%% skidding eval
%beta

skid = v_f .* cos(alpha+delta_f) - v_r .* cos(delta_r);
figure
plot(time,skid, "k", "LineWidth",line_width)
title('Skidding Factor')
xlabel('time (s)')
ylabel('Skidding (m/s)')
xlim([0 time(end)]); ylim([-0.2 0.2])
grid on


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
% pred = pred_k(2:end,:);
% ref = ref_k(2:end,:);
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
%         plot(ref(1+(k-1)*(N-1):k*(N-1),1),ref(1+(k-1)*(N-1):(N-1)*k,2), 'g--*') %ref
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

% % scatter(pred(N:N*2,1), pred(N:N*2,2))