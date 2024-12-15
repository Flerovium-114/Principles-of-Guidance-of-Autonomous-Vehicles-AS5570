function dy_dt = PP(t,y)
   
    V_T = 50;
    w_T = 0;
    V_P = 75;
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
    dy_dt(2) = (V_T*sin(alpha_T - theta) - V_P*sin(alpha_P - theta))/R;   
    k = 0;
    aP = V_P*dy_dt(2) - k*(alpha_P - theta);
    dy_dt(5) = aP/V_P;                      
    dy_dt(6) = 0; 
    %dy_dt(3) = -(V_theta/R)*(V_R + V_P);          
    dy_dt(4) = -V_T*sin(alpha_T - theta)*(dy_dt(6) - dy_dt(2)) + V_P*sin(alpha_P - theta)*(dy_dt(5) - dy_dt(2));      
    dy_dt(3) = V_T*cos(alpha_T - theta)*(dy_dt(6) - dy_dt(2)) - V_P*cos(alpha_P - theta)*(dy_dt(5) - dy_dt(2)); 
                             
    dy_dt(7) = V_T*cos(alpha_T);                % x_T_dot
    dy_dt(8) = V_T*sin(alpha_T);                % y_T_dot
    dy_dt(9) = V_P*cos(alpha_P);                % x_P_dot
    dy_dt(10) = V_P*sin(alpha_P);               % y_P_dot
end