%pivot + diff driven wheels
clearvars; clc; close all;
%% loading
input_path = fullfile(pwd, '../../position_controller/5_dof/passive/input.txt'); % Paths Folder
load(input_path)
state_path = fullfile(pwd, '../../position_controller/5_dof/passive/state.txt'); % Paths Folder
load(state_path)
prediction_path = fullfile(pwd, '../../position_controller/5_dof/passive/prediction.txt'); % Paths Folder
pred = load(prediction_path);
reference_path = fullfile(pwd, '../../position_controller/5_dof/passive/reference.txt'); % Paths Folder
ref = load(reference_path);
path_path = fullfile(pwd, '../../../path/std_path.txt'); % Paths Folder
path = load(path_path);
% TO DO add plot effort and compare to pivot active

%% data
dt = 0.1;
N = 10;
time = linspace(0,dt*length(input(:,1)),length(input(:,1)))';
lf = 0.16; lr = 0.71;
v_fl = input(:,1);
v_fr = input(:,2);
v_rl = input(:,3);
v_rr = input(:,4);
alpha = input(:,9);

%state
x = state(:,1);
y = state(:,2);
psi = state(:,3);

%path
x_p = path(:,1);
y_p = path(:,2);

figure
plot(x,y); hold on 
plot(x_p, y_p)
grid on
title('trajectory')
xlabel('X [m]')
ylabel('Y [m]')
legend('state', 'reference', 'Location','northwest')

%% plotting
%velocities
figure
subplot(221)
plot(v_fl); grid on
subplot(222)
plot(v_fr); grid on
subplot(223)
plot(v_rl); grid on
subplot(224)
plot(v_rr); grid on

%alpha
figure 
plot(alpha); grid on




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