clearvars; clc; close all;
%%loading
load('ackinput.txt')
load('ackstate.txt')
load('s_shape_path.txt')
%data
dt = 0.1;
lf = 0.16; lr = 0.71;
v_f = 0.0;
v_r = ackinput(:,2);
delta_f = ackinput(:,3);
delta_r = 0.0;
alpha = 0.0;
beta = atan2((lf .* tan(delta_r) + lr .* (tan(delta_f).*cos(alpha) + sin(alpha))), ...
     (lf + lr.*(cos(alpha) - tan(delta_f).*sin(alpha))));
%state
x = ackstate(:,1);
y = ackstate(:,2);
psi = ackstate(:,3);

%path
x_p = s_shape_path(:,1);
y_p = s_shape_path(:,2);

figure
plot(x,y); hold on 
plot(x_p, y_p)
grid on
title('trajectory')
xlabel('X [m]')
ylabel('Y [m]')
legend('state', 'reference', 'Location','northwest')

%% plotting
figure
subplot(211)
plot(v_r); grid on
subplot(212)
plot(delta_f)
grid on


%% rate of change 
omega_f = diff(delta_f);
en_coeff = sum(abs(omega_f)*dt);
disp(en_coeff)
figure
plot(omega_f); title('front', en_coeff)


%% skidding eval
%beta

skid = v_f .* cos(alpha+delta_f) - v_r .* cos(delta_r);
figure
plot(skid)
title('skidding')
ylim([-0.5 0.5])
