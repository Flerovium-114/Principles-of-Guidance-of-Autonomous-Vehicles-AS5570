function dy_dt = TPN(t,y,y0)
    b = 0;
    V_P0 = y0(11);
    V_T0 = y0(12);
    alpha_P0 = y0(5);
    alpha_T0 = y0(6);
    V_R0 = y0(4);
    V_theta_0 = y0(3);
    c = -3*V_R0;
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
    V_P = y(11);
    V_T = y(12);

    dy_dt = zeros(12, 1);

    dy_dt(1) = V_T*cos(alpha_T - theta) - V_P*cos(alpha_P - theta);  
    dy_dt(2) =  (V_T*sin(alpha_T - theta) - V_P*sin(alpha_P - theta))/R;       
    
    N = 3;
    %aP = c*V_theta/R;  % TPN
    aP = -N * V_R * dy_dt(2); % RTPN

    %if aP > 100

    aT = -30;


%     if aT > 10
%         aT = 10;
%     elseif aT < -10
%         aT = -10;
%     end

    dy_dt(5) = aP*cos(alpha_P - theta)/V_P;                          
    dy_dt(6) = aT/V_T; %aT*cos(alpha_T - theta)/V_T;  
    
    dy_dt(4) = -V_T*sin(alpha_T - theta)*(dy_dt(6) - dy_dt(2)) + V_P*sin(alpha_P - theta)*(dy_dt(5) - dy_dt(2));      
    dy_dt(3) = V_T*cos(alpha_T - theta)*(dy_dt(6) - dy_dt(2)) - V_P*cos(alpha_P - theta)*(dy_dt(5) - dy_dt(2));  
  
    dy_dt(7) = V_T*cos(alpha_T);                
    dy_dt(8) = V_T*sin(alpha_T);                
    dy_dt(9) = V_P*cos(alpha_P);                
    dy_dt(10) = V_P*sin(alpha_P);    

    dy_dt(11) = aP*sin(alpha_P - theta);
    dy_dt(12) = 0; % aT*sin(alpha_T - theta);

end