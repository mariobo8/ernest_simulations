clearvars; clc; close all;
%%loading
load('fwsinput.txt')
load('fwsstate.txt')
load('std_path.txt')
load('fwstime_step.txt')
load('fwsinput.txt')
pred_k = load('fwspred.txt');
ref_k = load ('fwsref.txt');
%%data
N = 10;
dt = 0.1;
lf = 0.16; lr = 0.71;
time = linspace(0,dt*length(fwsinput(:,1)),length(fwsinput(:,1)))';
coeff = 5;
v_f = movmean(fwsinput(:,1),coeff);
v_r = movmean(fwsinput(:,2),coeff);
delta_f = movmean(fwsinput(:,3),coeff);
delta_r = movmean(fwsinput(:,4),coeff);
alpha = movmean(fwsinput(:,5),coeff);
virtual_v = movmean(fwsinput(:,6),coeff);
%state
x = fwsstate(:,1);
y = fwsstate(:,2);
psi = fwsstate(:,3);

%path
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
plot(time, v_f*0.14, "k", "LineWidth", line_width); 
grid on; xlim([0, time(end)]); ylim([-0.2, 1]);
xlabel('time (s)'); ylabel('v_f (m/s)')
subplot(322)
plot(time, v_r*0.14, "r", "LineWidth", line_width); 
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
plot(time, virtual_v, "k", "LineWidth", line_width); 
grid on; xlim([0, time(end)]); ylim([-.2,1])
xlabel('time (s)'); ylabel('v_{virtual} (m/s)')

%% rate of change
omega_f = [0; diff(delta_f)];
omega_r = [0; diff(delta_r)];


en_coeff = sum((abs(omega_f)*2 + abs(omega_r)*2)*dt);
figure
subplot(211)
sgtitle(num2str(en_coeff))
plot(time, omega_f,'k', 'LineWidth',line_width)
grid on; xlim([0, time(end)]); ylim([-0.09 0.09])
xlabel('time (s)'); ylabel('\omega_f (m/s)')
subplot(212)
plot(time, omega_r,'k', 'LineWidth',line_width)
grid on; xlim([0, time(end)]); ylim([-0.09 0.09])
xlabel('time (s)'); ylabel('\omega_r (m/s)')


disp(en_coeff)



%% skidding eval
%beta

skid = v_f .* cos(delta_f) - v_r .* cos(delta_r);
figure
plot(time,skid, "k", "LineWidth",line_width)
title('Skidding Factor')
xlabel('time (s)')
ylabel('Skidding (m/s)')
xlim([0 time(end)]); ylim([-0.2 0.2])
grid on

%% time step

figure
fwstime_step_ms = fwstime_step .* 1000;
avg_step = mean(fwstime_step_ms);
plot(fwstime_step_ms,"k" ,"LineWidth",line_width)
hold on
yline(avg_step,"-r","LineWidth",line_width)
grid on 
xlim([0 length(time)]); ylim([0 150])
xlabel('MPC Step'); ylabel('Solving Time (ms)')





%% trajectory plotting

figure(500)

% Animate the robot motion
%figure;
set(gcf,'Position',[25 25 920 720]);
%set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
%set(gcf,'Units','normalized','OuterPosition',[10 0 0.55 1]);
    plot(x_p, y_p,'-k','linewidth',1);
    hold on
line_width = 1.5;
fontsize_labels = 15;
pred = pred_k(2:end,:);
ref = ref_k(2:end,:);
x_r_1 = [];
y_r_1 = [];

for k = 1:size(pred,1)
    h_t = 0.25; w_t=0.12; % triangle parameters
    %plot path
    plot(x_p, y_p,'-k','linewidth',1);
    hold on;
    if (1+(k-1)*(N+1)) > size(pred,1)
        break
    end
    x1 = pred(1+(k-1)*(N+1),1); y1 = pred(1+(k-1)*(N+1),2); th1 = pred(1+(k-1)*(N+1),3); %state
    x_r_1 = [x_r_1 x1];
    y_r_1 = [y_r_1 y1];

    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];

    plot(x_r_1,y_r_1,'-r','linewidth',line_width);hold on % plot exhibited trajectory
    if k < size(pred,1) % plot prediction
        plot(pred(1+(k-1)*(N+1):(N+1)*k,1),pred(1+(k-1)*(N+1):k*(N+1),2),'r--*') %pred
        hold on
        plot(ref(1+(k-1)*(N-1):k*(N-1),1),ref(1+(k-1)*(N-1):(N-1)*k,2), 'g--*') %ref
    end

    hold on
    fill(x1_tri, y1_tri, 'blue'); % plot robot position
    hold off
    ylabel('$y$ (m)','interpreter','latex','FontSize',fontsize_labels)
    xlabel('$x$ (m)','interpreter','latex','FontSize',fontsize_labels)
    legend('Path','Executed','Predicted', 'reference')
    axis([-12  0.8 -1 5.5]) 
    pause(0.001)
    box on;
    grid on
    %aviobj = addframe(aviobj,gcf);
    drawnow
    F(k) = getframe(gcf); % to get the current frame
end
close(gcf)
video = VideoWriter('fws_flat_path.avi','Uncompressed AVI');
video.FrameRate = 10;  % (frames per second) 
open(video)
writeVideo(video,F)
close (video)
