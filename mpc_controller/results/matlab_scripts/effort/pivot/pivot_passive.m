%pivot + diff driven wheels
clearvars; clc; close all;
%% loading
input_path = fullfile(pwd, '../../../pivot/passive/input.txt'); % Paths Folder
load(input_path)
bicycle = fullfile(pwd, '../../../pivot/passive/input_bicycle.txt'); % Paths Folder
load(bicycle)
state_path = fullfile(pwd, '../../../pivot/passive/state.txt'); % Paths Folder
load(state_path)
prediction_path = fullfile(pwd, '../../../pivot/passive/prediction.txt'); % Paths Folder
pred = load(prediction_path);
reference_path = fullfile(pwd, '../../../pivot/passive/reference.txt'); % Paths Folder
ref = load(reference_path);
path_path = fullfile(pwd, '../../../../path/new_path.txt'); % Paths Folder
path = load(path_path);
load(fullfile(pwd, '../../../pivot/passive/steer_effort.txt'))
load(fullfile(pwd, '../../../pivot/passive/wheel_effort.txt'))
% TO DO add plot effort and compare to pivot active

%%data
dt = 0.1;
N = 10;
time = linspace(0,dt*length(input(:,1)),length(input(:,1)))';
lf = 0.16; lr = 0.71;
v_fl = input(:,1);
v_fr = input(:,2);
v_rl = input(:,3);
v_rr = input(:,4);
alpha = input(:,9);

%bicycle
v_f = input_bicycle(:,1);
v_r = input_bicycle(:,2);
alpha = input_bicycle(:,5);

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


%% bicycle input
line_width = 1.5;
figure
subplot(311)
plot(time, v_f, "LineWidth", line_width); 
grid on; xlim([0, time(end)]); ylim([-0.2, 0.8]);
xlabel('time (s)'); ylabel('v_{f} (m/s)')
subplot(312)
plot(time, v_r, "LineWidth", line_width); 
grid on; xlim([0, time(end)]); ylim([-0.2, 0.8]);
xlabel('time (s)'); ylabel('v_{r} (m/s)')
subplot(313)
plot(time, alpha, "LineWidth", line_width); 
grid on; xlim([0, time(end)]); ylim([-0.6, 0.6]);
xlabel('time (s)'); ylabel('\alpha (rad)')

%% input
line_width = 1.5;
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

%alpha
figure 
plot(time, alpha, "b", "LineWidth", line_width);
grid on; xlim([0, time(end)]); ylim([-1.2, 1.2]);
xlabel('time (s)'); ylabel('\alpha (rad)')
legend('alpha', Orientation='horizontal')

%% STEERING TORQUE
%NAME torque
%TODO: change labels add filter and limits
f_torque = time(end)/length(steer_effort(:,1));
t_torque_m = 0:f_torque:time(end);
t_torque = t_torque_m(1:end-1);

line_width = 1;
figure
plot(t_torque, steer_effort(:,5), "k", "LineWidth", line_width); 
grid on; xlim([0, time(end)]); 
xlabel('time (s)'); ylabel('Steering Torque (Nm)')


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

%% energy computation (T *pivot_steering or T * )
%steer
tk = 0.05; %Nm/A torque constant
V = 80;
st_eff_new = interp1(t_torque', steer_effort(:,5), time);
nan_indices = isnan(st_eff_new);
st_eff_new(nan_indices) = 0;
I = st_eff_new / tk; %current
P_st = abs(V*I*1e-3); %power KW
E_st = trapz(time, P_st) / 3600

% %wheel 
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

E_tot = E_st + E_wheel

figure
plot(time, P_st)
xlabel('time (s)'); ylabel('Power (W)')
legend('\alpha','Orientation','horizontal')
grid on


% %% trajectory plotting
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
% predict = pred(2:end,:);
% reference = ref(2:end,:);
% x_r_1 = [];
% y_r_1 = [];
% 
% for k = 1:size(predict,1)
%     h_t = 0.25; w_t=0.12; % triangle parameters
%     %plot path
%     plot(x_p, y_p,'-k','linewidth',1);
%     hold on;
%     if (1+(k-1)*(N+1)) > size(predict,1)
%         break
%     end
%     x1 = predict(1+(k-1)*(N+1),1); y1 = predict(1+(k-1)*(N+1),2); th1 = predict(1+(k-1)*(N+1),3); %state
%     x_r_1 = [x_r_1 x1];
%     y_r_1 = [y_r_1 y1];
% 
%     x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
%     y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];
% 
%     plot(x_r_1,y_r_1,'-r','linewidth',line_width);hold on % plot exhibited trajectory
%     if (N+1)*k < size(predict,1) % plot prediction
%         plot(predict(1+(k-1)*(N+1):(N+1)*k,1),predict(1+(k-1)*(N+1):k*(N+1),2),'r--*') %pred
%         hold on
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