function [u_c] = obstacleCorrection(q, obstacle_pos)
    %obstacleCorrection computes the correction in the linear and angular velocities
    %input:  -q:[coloum vector 3x1] robot generalized coordinates
    %        -obstacle_pos:[cell array of vectors 1x2] list of positions of obstacles
    %output: -u_c:[coloum vector 2x1] the linear and angular correction velocities
    
    d_max = 0.8;    % m
    d_min = 0.1;    % m
    F_max = 2.2;    
    s = 3;
    c = F_max / (d_max - d_min)^s;
    I = 0.15;   % N.s^2/rad
    B = 1.3;    % N.s/rad
    K = 1.2;    % N/m
    Z = I*s^2 + B*s + K;
    
    F_R = 0.0;      F_L = 0.0;
    angle_R = 0.0;  angle_L = 0.0;
    
    d_R = zeros(1, size(obstacle_pos, 2)); %[0, 0]
    d_L = zeros(1, size(obstacle_pos, 2));
    
    alpha_R = zeros(1, size(obstacle_pos, 2));
    alpha_L = zeros(1, size(obstacle_pos, 2));
    
    for ind = 1:size(obstacle_pos, 2)
        obs = obstacle_pos{ind};
        d = sqrt(sum((obs - q(1:2)') .^ 2));
        alpha = atan2 ( (obs(2) - q(2)) , (obs(1) - q(1)) );
        if d > d_max || d < d_min || alpha < 0.0
            d_R(ind) = nan;     alpha_R(ind) = nan;
            d_L(ind) = nan;     alpha_L(ind) = nan;
        elseif alpha < 1.5708
            d_R(ind) = d;       alpha_R(ind) = 1.5708 - abs(q(3) - alpha);
            d_L(ind) = 0.0;     alpha_L(ind) = 0.0;
        else
            d_R(ind) = 0.0;     alpha_R(ind) = 0.0;
            d_L(ind) = d;       alpha_L(ind) = 1.5708 - abs(q(3) - alpha);
        end
    end
    
    if min(d_R) < d_max
        F_R = c*(d_max - d)^s;
        angle_R = alpha_R( d_R == min(d_R) );
    end
    
    if min(d_L) < d_max
        F_L = c*(d_max - d)^s;
        angle_L = alpha_L( d_L == min(d_L) );
    end  
    
    v_c = ( F_R*sin(angle_R) + F_L*sin(angle_L) ) / Z;
    w_c = (F_R - F_L) / Z;
    disp('v_c= ')
    disp(v_c)
    disp('w_c= ')
    disp(w_c)
    u_c = [v_c; w_c];
    %disp(u_c)
    %disp(size(u_c));
end
