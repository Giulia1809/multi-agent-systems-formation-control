function [u_c] = obstacleCorrection_fixed(q, obstacle_pos)
    %obstacleCorrection computes the correction in the linear and angular velocities
    %input:  -q:[coloum vector 3x1] robot generalized coordinates
    %        -obstacle_pos:[cell array of vectors 1x2] list of positions of obstacles
    %output: -u_c:[coloum vector 2x1] the linear and angular correction velocities

    d_max = 2.5;    % m
    d_min = 1.2;    % m
    F_max = 2.5;
    s = 3;
    c = F_max / (d_max - d_min)^s;
    I = 0.15;   % N.s^2/rad
    B = 1.3;    % N.s/rad
    K = 1.2;    % N/m
    Z = I*s^2 + B*s + K;
    
    d_R = 0.0;     d_L = 0.0;
    F_R = 0.0;     F_L = 0.0;
    alpha_R = 0.0; alpha_L = 0.0;
    
    for i = 1:size(obstacle_pos, 2) %get every 2D obstacle
        obs = obstacle_pos{i}; %get i-th obstacle
        d = norm(obs' - q(1:2));
        
        alpha = atan2 ((obs(2) - q(2)) , (obs(1) - q(1))) - q(3);
        if d >= d_max || alpha < -pi/2 || alpha > pi/2
            continue
        elseif alpha < 0
            d_R = d; 
            alpha_R = pi/2 + alpha; 
            F_R = c * (d_max - d_R)^s; 
        elseif alpha > 0
            d_L = d;
            alpha_L = pi/2 - alpha;
            F_L = c * (d_max -d_L)^s;
        end  
    end
    v_c = (F_R * sin(alpha_R) + F_L * sin(alpha_L))/Z;
    w_c = (F_R - F_L)/Z;
    
    u_c = [v_c; w_c];
end
