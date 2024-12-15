clc;
clear;
close all;

t_step = 0.1;
t_end = 1000;
t_span = 0:t_step:t_end;
t_terminate = 5;
options = odeset('Events', @(t, y) event_terminal_LOS(t, y));

%% LOS
R_T0 = 7000;
R_P0 = 100;
alpha_T0 = 45*pi/180;
alpha_P0 = 60*pi/180;
theta_P0 = 15*pi/180;
theta_T0 = 30*pi/180;
x_P0 = R_P0*cos(theta_P0);
y_P0 = R_P0*sin(theta_P0);
x_T0 = R_T0*cos(theta_T0);
y_T0 = R_T0*sin(theta_T0);
V_P = 200;
nu = 1.2;
V_T = V_P/nu;
V_R_P0 = V_P*cos(alpha_P0 - theta_P0);
V_theta_P_0 = V_P*sin(alpha_P0 - theta_P0);
V_R_T0 = V_T*cos(alpha_T0 - theta_T0);
V_theta_T_0 = V_T*sin(alpha_T0 - theta_T0);
V_R0 = V_R_T0 - V_R_P0;
V_theta0 = V_theta_T_0 - V_theta_P_0;

y0 = [R_P0, theta_P0, R_T0, theta_T0, V_R_P0, V_theta_P_0, V_R_T0, V_theta_T_0, alpha_P0, alpha_T0, x_P0, y_P0, x_T0, y_T0];
[t,y] = ode45(@(t,y) CLOS(t, y, V_P, V_T, R_P0, theta_P0, R_T0, theta_T0, V_R_P0, V_theta_P_0, V_R_T0, V_theta_T_0, alpha_P0, alpha_T0, x_P0, y_P0, x_T0, y_T0), t_span, y0, options);

K = 1;
%aP = K*R_P*(theta_T - theta_P) + R_P*d2_theta_T_dt2 + 2*dy_dt(1)*dy_dt(4);
%theta_ddot = diff(y(:, 4))
a = diff(y(:, 4));

theta_ddot = [a; a(end)];
aP = K.*y(:, 1).*(y(:, 4) - y(:, 2)) + (1/t_step).*y(:, 1).*theta_ddot + 2.*y(:, 5).*y(:, 8);

figure;
plot(aP,t);
title('aP vs t');
xlabel('t');
ylabel('aP')


% Extract the trajectories
x_T = y(:, 13);  
y_T = y(:, 14);  
x_P = y(:, 11); 
y_P = y(:, 12); 


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

figure;

plot(t, y(:, 1) - y(:, 3));
xlabel('time');
ylabel('R_{PT}');
title('separation vs time')






