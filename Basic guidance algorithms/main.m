% Remember to check both main and individual files for changes in initial
% conditions

clc;
clear;
close all;

R_0 = 500;
theta_0 = 0;
x_P0 = 0;
y_P0 = 0;
x_T0 = R_0*cos(theta_0);
y_T0 = R_0*sin(theta_0);

t_step = 0.1;

t_end = 100;
t_span = 0:t_step:t_end;
t_terminate = 5;
options = odeset('Events', @(t, y) event_terminal(t, y));

%% PP
nu = 1;
V_T = 50;
V_P = 75;
w_T = 0;
alpha_P0 = pi/4;
alpha_T0 = 0;
V_R0 = V_T*cos(alpha_T0 - theta_0) - V_P;
V_theta_0 = V_T*sin(alpha_T0 - theta_0);
y0 = [R_0, theta_0, V_theta_0, V_R0, alpha_P0, alpha_T0, x_T0, y_T0, x_P0, y_P0];
[t,y] = ode45(@PP, t_span, y0, options);
k = 0;
  % PP, DPP
%min(y(:, 1)); % miss- distance

%% DPP
nu = 1.2;
V_T = 300;
V_P = nu*V_T;
alpha_P0 = 55*pi/180;
alpha_T0 = 170*pi/180;
delta = alpha_P0 - theta_0;
V_R0 = V_T*cos(alpha_T0 - theta_0) - V_P*cos(delta);
V_theta_0 = V_T*sin(alpha_T0 - theta_0) - V_P*sin(delta);
y0 = [R_0, theta_0, V_theta_0, V_R0, alpha_P0, alpha_T0, x_T0, y_T0, x_P0, y_P0];
[t,y] = ode45(@DPP, t_span, y0, options);

aP = V_P.*y(:, 3)./y(:, 1);   % PP, DPP

%% PPN
N = 3;
nu = 1.2;
V_P = 400;
V_T = V_P/nu;
alpha_P0 = 50*pi/180;
alpha_T0 = 60*pi/180;
V_R0 = V_T*cos(alpha_T0 - theta_0) - V_P*cos(alpha_P0 - theta_0);
V_theta_0 = V_T*sin(alpha_T0 - theta_0) - V_P*sin(alpha_P0 - theta_0);
y0 = [R_0, theta_0, V_theta_0, V_R0, alpha_P0, alpha_T0, x_T0, y_T0, x_P0, y_P0];
[t,y] = ode45(@PPN, t_span, y0, options);

aP = N.*V_P.*y(:, 3)./y(:, 1); % PPN

%% TPN
b = 0;
nu = 1.5;
V_P0 = 400;
V_T0 = 250 ; %V_P0/nu;
alpha_P0 = 40*pi/180;
alpha_T0 = 170*pi/180;
V_R0 = V_T0 * cos(alpha_T0 - theta_0) - V_P0 * cos(alpha_P0 - theta_0);
V_theta_0 = V_T0 * sin(alpha_T0 - theta_0) - V_P0 * sin(alpha_P0 - theta_0);
c = -3*V_R0;
disp(V_theta_0^2 + V_R0^2 + 2*c*V_R0);
y0 = [R_0, theta_0, V_theta_0, V_R0, alpha_P0, alpha_T0, x_T0, y_T0, x_P0, y_P0, V_P0, V_T0];
[t,y] = ode45(@(t,y)TPN(t,y,y0), t_span, y0, options);
%aP = c.*y(:, 3)./y(:, 1); % TPN
N = 3;
aP = -N .* y(:, 4) .* y(:, 3)./y(:, 1);


%% PLOTS AND ANIMATION

R = y(:, 1);
min(y(:, 1)) % miss- distance
t(end)

[R_min, R_minIndex] = min(R);
t(R_minIndex)

figure;
plot(t, y(:, 1));
xlabel('Time (s)');
ylabel('R');

title('R over time');
grid on;

figure;
plot(t, y(:, 2));
xlabel('Time (s)');
ylabel('theta');
title('theta over time');
grid on;

figure;
plot(t, y(:, 3));
xlabel('Time (s)');
ylabel('v_theta');
title('v_theta over time');
grid on;

figure;
plot(t, y(:, 4));
xlabel('Time (s)');
ylabel('v_R');
title('v_R over time');
grid on;

figure;
plot(t, y(:, 5));
xlabel('Time (s)');
ylabel('alpha_p');
title('alpha_p over time');
grid on;

figure;
plot(t, y(:, 6));
xlabel('Time (s)');
ylabel('alpha_t');
title('alpha_t over time');
grid on;

theta_dot = (1/t_step).* diff(y(:, 2));
figure;
plot(t(1:end-1), theta_dot);
xlabel('t');
ylabel('theta_{dot}')
title('theta_{dot} vs time')

alpha_P = y(:, 5);
theta = y(:, 2);

k = 0;
aP = V_P.*theta_dot - k.*(alpha_P(1:end-1) - theta(1:end-1)); 


%aP = c.*y(:, 3)./y(:, 1); % TPN
%aP = V_P.*y(:, 3)/y(:, 1);   % PP, DPP
%aP = N*V_P*y(:, 3)./y(:, 1); % PPN

figure;
plot(t(1:end-1), aP)
xlabel('t');
ylabel('a_p');
title('a_p vs t');
grid on;
 
r = y(:, 1);
Vtheta = r(1: end-1).*theta_dot;
figure;
plot(t(1:end-1), Vtheta);
xlabel('t');
ylabel('V_{theta}');
title('v_{theta} vs time')


vr = y(:, 4);

plot(Vtheta, vr(1:end-1));
xlabel('v_{theta}')
ylabel('v_{r}')
title('v_{r} vs v_{theta}')

%aT = b./y(:, 3); % TPN
% figure;
% plot(t, aT)
% xlabel('t');
% ylabel('a_T');
% title('a_T vs t');
% grid on;

% 
% figure;
% plot(t, y(:, 11));
% xlabel('t');
% ylabel('V_P');
% title('V_P vs t');
% grid on;

% figure;
% plot(t, y(:, 12));
% xlabel('t');
% ylabel('V_T');
% title('V_T vs t');
% grid on;
% 
% figure;
% plot(t, y(:, 11));
% xlabel('t');
% ylabel('V_p');
% title('V_p vs t');
% grid on;


% Extract the trajectories
x_T = y(:, 7);  
y_T = y(:, 8);  
x_P = y(:, 9); 
y_P = y(:, 10); 

%Plotting the initial positions

figure;
hT = plot(x_T(1), y_T(1), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b'); % Target point
hold on;
hP = plot(x_P(1), y_P(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % Pursuer point
hTrajT = plot(x_T(1), y_T(1), 'b-', 'LineWidth', 1.5); % Target trajectory
hTrajP = plot(x_P(1), y_P(1), 'r-', 'LineWidth', 1.5); % Pursuer trajectory
xlabel('x');
ylabel('y');
title('Trajectories of Target (T) and Pursuer (P)');
%legend('Target (T)', 'Pursuer (P)');
grid on;
axis equal;
hold off;

% Animation loop
for i = 1:length(t)
    % Update target position
    set(hT, 'XData', x_T(i), 'YData', y_T(i));
    % Update pursuer position
    set(hP, 'XData', x_P(i), 'YData', y_P(i));
    % Update trajectories
    set(hTrajT, 'XData', x_T(1:i), 'YData', y_T(1:i));
    set(hTrajP, 'XData', x_P(1:i), 'YData', y_P(1:i));
    
    % Pause to control animation speed
    pause(0.01);
end
figure;
plot(x_P, y_P);
hold on;
plot(x_T, y_T);

title('Trajectory');
xlabel('x');
ylabel('y');
legend('Pursuer','Target');
grid on;
