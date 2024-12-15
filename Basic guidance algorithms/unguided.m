function dy_dt = unguided(t,y)
    nu = 1.2;
    V_T = 150;
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

    dy_dt = zeros(10, 1);

    dy_dt(1) = V_T*cos(alpha_T - theta) - V_P*cos(alpha_P - theta);
    %dy_dt(1) = V_R;
    dy_dt(2) = (V_T*sin(alpha_T - theta) - V_P*sin(alpha_P - theta))/R;
    %dy_dt(2) = V_theta/R;
    dy_dt(3) = -V_theta*V_R/R;
    dy_dt(4) = V_theta^2/R;
    dy_dt(5) = 0;
    dy_dt(6) = 0;
    dy_dt(7) = V_T*cos(alpha_T);
    dy_dt(8) = V_T*sin(alpha_T);
    dy_dt(9) = V_P*cos(alpha_P);
    dy_dt(10) = V_P*sin(alpha_P);
end