function dy_dt = CLOS(t,y, V_P, V_T, R_P0, theta_P0, R_T0, theta_T0, V_R_P0, V_theta_P_0, V_R_T0, V_theta_T_0, alpha_P0, alpha_T0, x_P0, y_P0, x_T0, y_T0)

R_P = y(1);
theta_P = y(2);
R_T = y(3);
theta_T = y(4);
V_R_P = y(5);
V_theta_P = y(6);
V_R_T = y(7);
V_theta_T = y(8);
alpha_P = y(9);
alpha_T = y(10);
x_P = y(11);
y_P = y(12);
x_T = y(13);
y_T = y(14);


dy_dt = zeros(size(y)); 
theta = theta_P;

% [R_P0, theta_P0, R_T0, theta_T0, V_R_P0, V_theta_P_0, V_R_T0, V_theta_T_0, alpha_P0, alpha_T0, x_P0, y_P0, x_T0, y_T0]
dy_dt(1) = V_P*cos(alpha_P - theta_P);
dy_dt(2) = V_P*sin(alpha_P - theta_P)/R_P;
dy_dt(3) = V_T*cos(alpha_T - theta_T);
dy_dt(4) = V_T*sin(alpha_T - theta_T)/R_T;

K = 1;
aT = 10;
dy_dt(10) = aT/V_T;
a_T_n = dy_dt(3)*dy_dt(10);
d2_theta_T_dt2 = (a_T_n - 2*dy_dt(3)*dy_dt(4))/(R_T);
aP = K*R_P*(theta_T - theta_P) + R_P*d2_theta_T_dt2 + 2*dy_dt(1)*dy_dt(4);

dy_dt(9) = aP/V_P;


dy_dt(5) = -V_P*sin(alpha_P - theta)*(dy_dt(9) - V_theta_P/R_P);
dy_dt(6) = V_P*cos(alpha_P - theta_P)*(dy_dt(9) - V_theta_P/R_P);
dy_dt(7) = -V_T*sin(alpha_T - theta_T)*(dy_dt(10) - V_theta_T/R_T);
dy_dt(8) = V_T*cos(alpha_T - theta_T)*(dy_dt(10) - V_theta_T/R_T);

dy_dt(11) = V_P*cos(alpha_P);
dy_dt(12) = V_P*sin(alpha_P);
dy_dt(13) = V_T*cos(alpha_T);
dy_dt(14) = V_T*sin(alpha_T);

end