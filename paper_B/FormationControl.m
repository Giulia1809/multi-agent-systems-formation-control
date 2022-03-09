classdef FormationControl < matlab.mixin.Copyable
    %controller for each follower robot
    
    properties 
        %constant values
        T;              %prection horizon of MPC, the control horizon is 
                        %   chosen equal to the prediction horizon
        Ts;  %used in collision avoidance discretization
        lambda;         %gain parameter in error dynamics (12)
        eta;            %upperbound in constraint (28)
        gamma;          %upperbound in constraint (29)
        psi;            %distance bound in (5), also in constraint (27)
        psi_big;        %parameter in distance constraints (6)
        d;              %offset of controlled position
        n;              %total number of followers
        i;              %robot number
        R;              %gain symmetric positive def matrix in (25)
 
        %variable values
        k;              %simulation time step
        e;              %actual error at step k
        e_hat;          %matrix 2xT containing predicted error trajectory
        alpha;          %matrix 2xT containing optimal control trajectory 
                        %   applied until next optimization
        mu;             %difference between reference and leader trajectory
    end
    
    methods
        function obj = FormationControl(T, Ts, lambda, eta, ...
                gamma, psi, psi_big, d, n, i, R, q, z_des, q_l)
            %Construct an instance of this class
            %input:  constant values of controller described above plus
            %        -q:[vector 3x1] initial generalized coordinates
            %        -z_des:[vector 2x1] initial desired controlled position 
            %        -q_l:[vector 3x1] initial leader robot configuration
            %output: -obj:[FormationControl] an instance of this class
                               
            obj.T = T;
            obj.Ts = Ts;
            obj.lambda = lambda;
            obj.eta = eta;
            obj.gamma = gamma;
            obj.psi = psi;
            obj.psi_big = psi_big;
            obj.d = d;
            obj.n = n;
            obj.i = i;
            obj.R = R;
            obj.d = d;
            
            obj.k = 0;
            z = [q(1) + obj.d * cos(q(3)); 
                 q(2) + obj.d * sin(q(3))];
            obj.e = z - z_des;
            obj.e_hat = [obj.e, obj.e, obj.e];
            obj.alpha = [lambda * obj.e, lambda * obj.e, lambda * obj.e];
            obj.mu = z_des - q_l(1:2);
        end
        
        function u = GetControl(obj, q, z_des, q_l, u_l, e_others, mu_others, constrained)
            %GetControl computes the next control [v, w]' to apply
            %input:  -q:[vector 3x1] robot configuration
            %        -z_des:[vector 2x1] desired controlled position
            %        -q_l:[vector 3x1] leader robot configuration
            %        -u_l:[vector 2x1] leader control input
            %        -e_others:[matrix 2x(n-1)] e value of other followers
            %        -mu_others:[matrix 2x(n-1)] mu value of other followers
            %        -constrained:[boolean] True if constrained MPC is computed
            %ouput:  -u:[coloum vector 2x1] linear and angular velocities
            
            if mod(obj.k, obj.n) == obj.i-1
                [obj.alpha, obj.e_hat] = obj.Optimize(e_others, mu_others, constrained);
            end
            
            %compute u
            F = [ cos(q_l(3)), - (z_des(2) -  q_l(2)); 
                  sin(q_l(3)),   z_des(1) -  q_l(1)]; %(8a) 

            G_z = [ cos(q(3)), - obj.d * sin(q(3)); 
                    sin(q(3)),   obj.d * cos(q(3))]; %(10a)

            u = inv(G_z) * (- obj.lambda * obj.e + F * u_l + obj.alpha(:, 1)); %(12)
        end
        
        function obj = Update(obj, z_des, q_l, q)
            %Updates performs needed updates for next step
            %input:  -z_des:[vector 2x1] desired controlled position 
            %        -q_l:[vector 3x1] leader robot configuration
            %        -q:[vector 3x1] robot configuration
            %output: -obj:[SIDE EFFECT] this method update k, mu and alpha 
            %             of the object
            
            obj.k = obj.k + 1;
            obj.mu = z_des - q_l(1:2);
            obj.alpha(:, 1:2) = obj.alpha(:, 2:3);
            obj.alpha(:, 3) = zeros(2, 1);
            obj.e = obj.GetZ(q) - z_des;
            obj.e_hat(:, 1:2) = obj.e_hat(:, 2:3);
            obj.e_hat(:, 3) = (1 - obj.lambda*obj.Ts)*obj.e_hat(:, 2);
        end
        
        function z = GetZ(obj, q)
            %compute controlled position z given current configuration
            %input:  -q:[vector 3x1] robot configuration
            %output: -z:[vector 2x1] controlled position with offset d
            
            z = [q(1) + obj.d * cos(q(3)); 
                 q(2) + obj.d * sin(q(3))]; %(4)
        end
     
    end
    
    methods(Access = private)
        
        function [alpha, e] = Optimize(obj, e_others, mu_others, constrained)
            %Optimize computes optimal [alpha, e]' used to compute u = [v, w]'
            %input:  -e_others:[matrix 2xT*(n-1)] e trajectories of other followers
            %        -mu_others:[matrix 2x(n-1)] mu value of other followers 
            %        -constrained:[boolean] True if constrained MPC is computed
            %output: -alpha[matrix 2xT] optimal control trajectory
            %        -e[vector 2x1] predicted error related to alpha

            % Define 1 - lambda*Ts as constant c to get simplified code
            c = (1 - obj.lambda*obj.Ts);

            % A matrix for equation 27 after discretization
            A_27 = zeros(15, 18);           
            % First column is initialised to utilize the pattern in defining other columns
            col = [obj.Ts  0  -obj.Ts  0  c*obj.Ts  0  -c*obj.Ts  0  (c^2)*obj.Ts  0 -(c^2)*obj.Ts 0];        
            ind = [1, 2, 5, 6, 9, 10];
            for col_ind = 1:6
                A_27(ind(col_ind):12, col_ind) = col(1:12-ind(col_ind)+1);
            end
            A_27(1:end-3, 7:end) = - eye(12) * obj.psi_big;
            A_27(13, 7:10) = ones(1, 4);
            A_27(14, 11:14) = ones(1, 4);
            A_27(15, 15:18) = ones(1, 4);

            % b vector for equation 27 after discretization
            b_27 = zeros(15, 1);
            for col_ind = 1:3
                b_27(col_ind*4 - 3 : col_ind*4 - 2, 1) = -(c^col_ind)*obj.e + e_others(:, col_ind) - obj.mu + mu_others - [obj.psi; obj.psi];
                b_27(col_ind*4 - 1 : col_ind*4,     1) =  (c^col_ind)*obj.e - e_others(:, col_ind) + obj.mu - mu_others - [obj.psi; obj.psi];
            end
            b_27(13) = 3;
            b_27(14) = 3;
            b_27(15) = 3;

            % A matrix for equation 28 after discretization
            A_28 = zeros(12, 18);
            A_28(:, 1:6) = A_27(1:12, 1:6);

            % b vector for equation 28 after discretization
            b_28 = zeros(12, 1);
            for col_ind = 1:3
                b_28(col_ind*4 - 3 : col_ind*4 - 2, 1) = -(c^col_ind)*obj.e + [obj.eta; obj.eta];
                b_28(col_ind*4 - 1 : col_ind*4,     1) =  (c^col_ind)*obj.e + [obj.eta; obj.eta];
            end

            % A matrix for equation 29 after discretization
            A_29 = zeros(4, 18);
            A_29(:, 1:6) = A_27(9:12, 1:6);
            
            % b vector for equation 29 after discretization
            b_29 = zeros(4, 1);
            b_29(1:2, 1) = -(c^3)*obj.e + [obj.gamma; obj.gamma];
            b_29(3:4, 1) =  (c^3)*obj.e + [obj.gamma; obj.gamma];

            if constrained == true
                % Final A matrix after stacking all the inequality constraints constrained
                A = zeros(31, 18);
                A(1:15, :) = A_27;
                A(16:27, :) = A_28;
                A(28:31, :) = A_29;
                
                % Final b vector after stacking all the inequality constraints constrained
                b = zeros(31, 1);
                b(1:15, :) = b_27;
                b(16:27, :) = b_28;
                b(28:31, :) = b_29;
            else
                % Final A matrix after stacking all the inequality unconstrained
                A = zeros(19, 18);
                A(1:15, :) = A_27;
                A(16:19, :) = A_29;

                % Final b vector after stacking all the inequality unconstrained
                b = zeros(19, 1);
                b(1:15, :) = b_27;
                b(16:19, :) = b_29;
            end

            H = zeros(18, 18);
            H(1:6, 1:6) = [obj.R      zeros(2,2) zeros(2,2);
                           zeros(2,2) obj.R      zeros(2,2);
                           zeros(2,2) zeros(2,2) obj.R      ];
            
            xtype = [7:18];
            [x,~,~,~]=miqp(H, zeros(18,1), A, b, [], [], xtype);
            alpha = reshape(x(1:6), [2, 3]);
            
            e = zeros(2, 3);
            e(:, 1) = (1 - obj.lambda*obj.Ts)*obj.e + obj.alpha(:, 1)*obj.Ts;
            e(:, 2) = (1 - obj.lambda*obj.Ts)*e(:,1) + obj.alpha(:, 2)*obj.Ts;
            e(:, 3) = (1 - obj.lambda*obj.Ts)*e(:,2) + obj.alpha(:, 3)*obj.Ts;
        end
            
    end
    
end
