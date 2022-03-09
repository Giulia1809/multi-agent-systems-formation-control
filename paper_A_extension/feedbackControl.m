function [v_f, w_f, xi_next, sym] = feedbackControl(A, B, Q, R, input_constraint, z_e_k, phi, xi_prev, T_s, n, m, p)
    %feedbackControl compute the feedback term in the control law
    % it computes the optimal control using MPC to minimize the error of
    % the linearized system after feedback linearization
    % At the end the orignal input signal is recovered by using the
    % formulas of the dynamic compensator
    %input:  -A:[matrix 2x2] state matrix of the linearized system
    %        -B:[matrix 2x2] input matrix of the linearized system
    %        -Q:[matrix 2x2] state cost matrix for MPC feedback
    %        -R:[matrix 2x2] input cost matrix for MPC feedback
    %        -z_e_k:[coloum vector 2x1] initial state error
    %        -phi:[scalar] orientation of the robot
    %        -xi_prev:[scalar] the state of the dynamic compensator 
    %        -T_s: sampling time in order to compute A
    %        and B of the discretized system via forward Euler
    %        -n, m: [two scalar] state and input dimension of the
    %        linearized system
    %        -p: prection horizon of MPC, the control horizon is chosen 
    %        equal the prediction horizon
    %output: -[v_f, w_f]:[scalar, scalar] feedback control action
    %        -xi_next:[scalar] the next state of the compensator
    %        -sym:[boolean] debug utily, return if the hessian is symmetric
    
    %compute A and B of the discretized system
    A_d = @(k) eye(n) + A * k * T_s;
    B_d = @(k) B * T_s;
    
    A_hat = zeros(n*p, n);
    B_hat = zeros(n*p, m*p);
    
    rho_k = A_d(0);
    for r = 1:p
        A_hat((r-1)*n+1:(r-1)*n+n, :) = rho_k;
        rho_k = rho_k * A_d(1); 
    end
    
    for i = 1:p
        B_i = B_d(i);
        side_of_A_hat = [eye(n, n); A_hat(i*n+1:end, :)];
        for j = 1:p-i+1
            %array starts from 1 -> sad world :'(
            B_hat((j-1)*n+1+(i-1)*n:j*n+(i-1)*n, (i-1)*m+1:i*m) = side_of_A_hat((j-1)*n+1:(j)*n, :) * B_i;
        end
    end
    
    Q_hat = kron(eye(p), Q);
    R_hat = kron(eye(p), R);
    
    %coefficent of the quadratic function
    H_k = 2 * (B_hat' * Q_hat * B_hat + R_hat);
    f_k = 2 * B_hat' * Q_hat * A_hat * z_e_k;
    
    sym = issymmetric(H_k);
    
    lb = kron(ones(p,1), input_constraint(:,1));
    ub = kron(ones(p,1), input_constraint(:,2));
    
    options = optimset('Display', 'off');
    u_hat = quadprog(H_k, f_k', [], [], [], [], lb, ub, [], options);
    
    %feed the dynamic compensator in order to obtain the original inputs
    dxi = u_hat(1) * cos(phi) + u_hat(2) * sin(phi);
    v_f = xi_prev + dxi * T_s;
    w_f = (u_hat(2) * cos(phi) - u_hat(1) * sin(phi)) / v_f;
    xi_next = v_f;
end

