clc
clear;
close all;

global nu c  b V_T0 V_P0; %#ok<*GVMIS> 


clc
% COMMON DATA
R_0 = 100;
theta_0 = 15*pi/180;
x_p0 = 0;
y_p0 = 0;
x_t0 = R_0*cos(theta_0);
y_t0 = R_0*sin(theta_0);

% TIME CONDITIONS
t_step = 0.1;
t_end = 100;
t_span = 0:t_step:t_end;
t_terminate = 5;

% ODE CONDITIONS
options = odeset('Events', @(t, y) event_terminal(t, y));


% TPN
nu = 1.2;
c = 1.2;
b = 0;
V_T0 = 10;
V_P0 = V_T0*nu;
alpha_P0 = 30*pi/180;
alpha_T0 = 35*pi/180;
V_R0 = V_T0 * cos(alpha_T0 - theta_0) - V_P0 * cos(alpha_P0 - theta_0); 
V_theta_0 = V_T0 * sin(alpha_T0 - theta_0) - V_P0 * sin(alpha_P0 - theta_0);

fprintf("V_theta = %6.3f \n", V_theta_0);

disp(V_theta_0^2 + V_R0^2 + 2*c*V_R0);
y0 = [R_0, theta_0, V_theta_0, V_R0, alpha_P0, alpha_T0, x_t0, y_t0, x_p0, y_p0, V_P0, V_T0];
[t,y] = ode45(@TPN, t_span, y0, options);

%% PLOTS AND ANIMATION

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

figure;
plot(y(:, 3), y(:, 4));
xlabel('V_{theta}');
ylabel('V_R');
title('V_R vs V_{theta}');
grid on;

 aP = c.*y(:, 3)./y(:, 1);
figure;
%plot(t, V_P*y(:, 3)/y(:, 1));
plot(t, aP)
xlabel('t');
ylabel('a_p');
title('a_p vs t');
grid on;
% 
% aT = b./y(:, 3);
% figure;
% plot(t, aT)
% xlabel('t');
% ylabel('a_T');
% title('a_T vs t');
% grid on;
% 
% 
% figure;
% plot(t, y(:, 11));
% xlabel('t');
% ylabel('V_P');
% title('V_P vs t');
% grid on;
% 
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
legend('Target (T)', 'Pursuer (P)');
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
