clc;
clear;
close all;
% clear two_p_PPN_without_mu2;
% clear main_ext;

%global cost ap_matrix scenario_num t_span;
global t_ori N_values;
N_values = [];

R_0 = 2500;
theta_0 = 0*pi/180;
x_p0 = 0;
y_p0 = 0;
x_t0 = R_0*cos(theta_0);
y_t0 = R_0*sin(theta_0);

%% TIME CONDITIONS
t_step = 0.1;
t_end = 500;
t_span = 0:t_step:t_end;
t_terminate = 5;
options = odeset('Events', @(t, y) event_terminal(t, y));

alpha_P_df = deg2rad(-150);
V_T = 0;
V_P = 50;
alpha_P0 = pi/4;
alpha_T0 = 0;
V_R0 = V_T*cos(alpha_T0 - theta_0) - V_P*cos(alpha_P0 - theta_0);
V_theta_0 = V_T*sin(alpha_T0 - theta_0) - V_P*sin(alpha_P0 - theta_0);
y0 = [R_0, theta_0, V_theta_0, V_R0, alpha_P0, alpha_T0, x_t0, y_t0, x_p0, y_p0];

N0 = (alpha_P_df - alpha_P0)/(alpha_P_df - theta_0);

N_ori = linspace(0,N0,10);
scenario_count = length(N_ori); 
%ap_matrix = nan(length(t_span),scenario_count);
%results = cell(1, scenario_count);   
cost = zeros(1, scenario_count);  


% Define terminal condition (example: stop when R reaches a certain threshold)
threshold_R = 0.01*R_0;  % Example threshold
terminal_condition = @(y) y(1) <= threshold_R;

for scenario_num = 1:scenario_count
    N_values = [];
    t_ori = 0;

    % Set up parameters
    params = [V_P, V_T, alpha_P_df, alpha_P0, theta_0,  N_ori(scenario_num)];

    %[t, y] = ode45(@(t, y) two_p_PPN_without_mu2(t, y, V_P, V_T, alpha_P_df,...
    % alpha_P0, theta_0, N_ori(scenario_num)), t_span, y0, options);
    
    % Call the solver
    y = rk4_solver(@two_p_PPN_without_mu2, y0, t_span, t_step, params, terminal_condition);

    if length(N_values) < length(t_span)
        N_values = [N_values; NaN(length(t_span) - length(N_values), 1)];
    end
    N_matrix(:, scenario_num) = N_values;

    R = y(:,1);
    theta = y(:,2);
    Vtheta = y(:,3);
    Vr = y(:,4);
    alphaP = y(:,5);
    alphaT = y(:,6);
    xt = y(:,7);
    yt = y(:,8);
    xp = y(:,9);
    yp = y(:,10);

    % Define the acceleration arrays over their respective ranges
    t_ori_idx = 1:ceil(t_ori/t_step);  % Use proper indexing range
    t_end_idx = ceil(t_ori/t_step)+1:length(R);  % Full span till end

    
    % Calculate accelerations
%   aP_ori = N_matrix(t_ori_idx(1:end-1), scenario_num).* diff(theta(t_ori_idx))* V_P ;
%   aP_2 = N_matrix(t_end_idx(1:end-1), scenario_num) .*diff(theta(t_end_idx))*V_P;
    aP_ori = N_matrix(t_ori_idx, scenario_num) .* V_P .* Vtheta(t_ori_idx) ./ R(t_ori_idx);
    aP_2 = N_matrix(t_end_idx, scenario_num) .* V_P .* Vtheta(t_end_idx) ./ R(t_end_idx);

    % Accumulate cost as the sum of squared accelerations over time
    cost(scenario_num) = t_step.*(sum(aP_ori.^2) + sum(aP_2.^2));

    % Combined plot: R, theta, V_theta, V_R
    figure;
    subplot(2, 2, 1); plot(t_span(1:length(R)), R); xlabel('Time (s)'); ylabel('Range (m)'); title('Range R');
    subplot(2, 2, 2); plot(t_span(1:length(R)), theta); xlabel('Time (s)'); ylabel('\theta (rad)'); title('LOS Angle \theta');
    subplot(2, 2, 3); plot(t_span(1:length(R)), Vtheta); xlabel('Time (s)'); ylabel('V_\theta (rad/s)'); title('Angular Velocity V_\theta');
    subplot(2, 2, 4); plot(t_span(1:length(R)), Vr); xlabel('Time (s)'); ylabel('V_R (m/s)'); title('Radial Velocity V_R');
    sgtitle('Combined Plots');
    % Plot cost vs theta_d
    figure;
    plot( t_span(1:length(R)),[aP_ori',aP_2']);
    xlabel('t');
    ylabel('aP');
    title('Lateral Accelaration');
    grid on;

    plot_real_time_chase(t_span(1:length(R)), R, theta, V_T, V_P, 0, alphaP)
end

% Plot cost vs theta_d
figure;
plot(N_ori, cost, '-o', 'LineWidth', 1.5);
xlabel('N');
ylabel('Cost');
title('Cost vs N');
grid on;


%--- Function for Real-Time Plotting of the Chase ---%
function plot_real_time_chase(t, R, theta, V_T, V_P, alpha_T, alpha_P)
    % Initialize positions of the pursuer and target
    pursuer_pos = [0, 0]; % Initial pursuer position at origin
    target_pos = [R(1) * cos(theta(1)), R(1) * sin(theta(1))]; % Initial target position

    % Arrays to store trajectory history
    pursuer_traj = pursuer_pos;
    target_traj = target_pos;
   
    % Create figure for real-time plotting
    figure(5);
    hold on;
    grid on;
    axis equal;
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Real-Time Pursuer and Target Trajectory');
    
    % Initialize plot handles for pursuer and target
    hPursuer = plot(pursuer_pos(1), pursuer_pos(2), 'bo', 'MarkerFaceColor', 'b', 'DisplayName', 'Pursuer');
    hTarget = plot(target_pos(1), target_pos(2), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Target');
    hPursuerTraj = plot(pursuer_traj(:,1), pursuer_traj(:,2), 'b--', 'DisplayName', 'Pursuer Trajectory');
    hTargetTraj = plot(target_traj(:,1), target_traj(:,2), 'r-', 'DisplayName', 'Target Trajectory');
    legend('show');
    
    % Loop through time steps for real-time updates
    for i = 2:length(t)
        % Update target position based on constant velocity
        dt = t(i) - t(i-1); % Time step
        target_pos = target_pos + [V_T * cos(alpha_T),  V_T * sin(alpha_T)] * dt;

        % Update pursuer position based on the velocity and time step
        %pursuer_pos = pursuer_pos + [V_P * cos(alpha_P(i)), V_P * sin(alpha_P(i))] * dt;
        pursuer_pos = target_pos - [R(i) * cos(theta(i)),  R(i) * sin(theta(i))];

        % Append current positions to trajectory history
        pursuer_traj = [pursuer_traj; pursuer_pos];
        target_traj = [target_traj; target_pos];

        % Ensure plot handles are valid before updating
        if isvalid(hPursuer) && isvalid(hTarget) && isvalid(hPursuerTraj) && isvalid(hTargetTraj)
            % Update plot positions and trajectories
            set(hPursuer, 'XData', pursuer_pos(1), 'YData', pursuer_pos(2));
            set(hTarget, 'XData', target_pos(1), 'YData', target_pos(2));
            set(hPursuerTraj, 'XData', pursuer_traj(:,1), 'YData', pursuer_traj(:,2));
            set(hTargetTraj, 'XData', target_traj(:,1), 'YData', target_traj(:,2));

            % Pause to create real-time effect
            %pause(1e-6);
        else
            % Exit loop if plot handles are no longer valid (e.g., figure closed)
            break;
        end
    end
end
