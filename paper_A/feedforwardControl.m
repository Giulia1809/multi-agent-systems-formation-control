function [v_r, w_r] = feedforwardControl(p_r, d_r, dd_r, direction)
    %feedforwardControl compute the feedforward term in the control law
    % the nominal control term which allows to follow the trajectory in the
    % presence of no initial error or disturbances 
    %input:  -p_r:[coloum vector 2x1] desired position (not used)
    %        -d_r:[coloum vector 2x1] desired velocities
    %        -dd_r:[coloum vector 2x1] desired acceleration 
    %        -direction:[string] forward or backward, direction of motion
    %output: -v_r:[scalar] nominal linear velocity
    %        -w_r:[scalar] nominal angular velocity
    
    v_r = sqrt(d_r' * d_r);
    if direction ~= "forward"
        v_r = -v_r;
    end
    w_r = (dd_r(2) * d_r(1) - dd_r(1) * d_r(2)) / (d_r' * d_r);
end

