function dy_dt = DPP(t,y)
    nu = 1.2;
    V_T = 300;
    V_P = nu*V_T;
    R = y(1);
    theta = y(2);
    V_theta = y(3);
    V_R = y(4);
    alpha_P = y(5);
    alpha_T = y(6);
    x_T = y(7);
    y_T = y(8);
    x_P = y(9);
    y_P = y(10);
    delta = 25*pi/180;

    dy_dt = zeros(10, 1);

    dy_dt(1) = V_T*cos(alpha_T - theta) - V_P*cos(delta);      % V_R
    dy_dt(2) = (V_T*sin(alpha_T - theta) - V_P*sin(delta))/R;  % theta_dot
    k = 0;
    aP = V_P*V_theta/R;% - k*(alpha_P - theta - delta);
    dy_dt(3) = -(V_theta/R)*(V_R + V_P*cos(delta));               % V_theta_dot
    dy_dt(4) = dy_dt(2)*(V_theta + V_P*sin(delta));            % V_R_dot
    dy_dt(5) = aP/V_P;                                       % alpha_P_dot
    dy_dt(6) = 0;                                              % alpha_T_dot
    dy_dt(7) = V_T*cos(alpha_T);                               % x_T_dot
    dy_dt(8) = V_T*sin(alpha_T);                               % y_T_dot
    dy_dt(9) = V_P*cos(alpha_P);                               % x_P_dot
    dy_dt(10) = V_P*sin(alpha_P);                              % y_P_dot
end