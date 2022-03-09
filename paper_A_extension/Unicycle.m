classdef Unicycle < matlab.mixin.Copyable
    %kinematic model of the differential driver robot
    
    properties
        q;      %[x, y, phi] generalized coordinates
        dq;     %[dx, dy, dphi] time derivatives 
        t;      %time
        h;      %distance between the vehicle longitudinal axis and each wheel
        
        input_constaint; %[u_min, u_max]input constaints 2X2 matrix
        
        %some information of the linearized system via dynamic feedback
        %linerization
        n;      %state dimension
        m;      %input dimension
        A;      %state matrix
        B;      %input matrix
        xi;     %compensator state (linear velocity of the robot)
    end
    
    methods
        function obj = Unicycle(varargin)
            %Construct an instance of this class
            %input:  -q:[coloum vector 3x1] initial generalized coordinates
            %        -h:[scalar][OPTIONAL] distance between the wheels
            %output: -obj:[Unicycle] an instance of this class
            
            obj.q = varargin{1};
            obj.dq = zeros(3, 1);
            obj.t = 0;
            obj.xi = 0;
            
            obj.input_constaint = [-0.5, 0.5;
                                   -0.5, 0.5];
            
            obj.n = 4;
            obj.m = 2;
            obj.A = zeros(obj.n, obj.n);
            obj.A(1:2, 3:4) = eye(2, 2);
            obj.B = zeros(obj.n, obj.m);
            obj.B(3:4, :) = eye(2, 2);
            
            if nargin > 1
                obj.h = varargin{2};
            else 
                obj.h = 0;
            end
        end
        
        function obj = Step(obj, input, delta_t)
            %Step performs the integration of the kinematic model using
            % Euler integration
            %input:  -input:[coloum vector 2x1] linear and angular velocities
            %        -delta_t:[scalar] time interval for integration
            %output: -obj:[SIDE EFFECT] this method update q and dq of the
            %             object
            
            obj.dq = obj.GetG * input;
            obj.q = obj.q + obj.dq * delta_t;
            obj.q(3) = wrapToPi(obj.q(3));
            obj.t = obj.t + delta_t;
        end
        
        function obj = Step1(obj, input, delta_t)
            %Step1 performs the integration of the kinematic model using
            % Runge-kutta integration
            %input:  -input:[coloum vector 2x1] linear and angular velocities
            %        -delta_t:[scalar] time interval for integration
            %output: -obj:[SIDE EFFECT] this method update q and dq of the
            %             object
            
            function dq = kinematic(t, q)
                obj.q = q;
                dq = obj.GetG * input;
                obj.dq = dq;
            end
            
            [t, q] = ode45(@kinematic, [obj.t obj.t + delta_t], obj.q);
            
            obj.t = t(end);
            q =  q(end, :)';
            obj.q = [q(1:2); wrapToPi(q(3))];
        end
        
        function [u, sym]= GetControl(obj, p_r, d_r, dd_r, direction, Q, R, sampling_time, prediction_horizon)
            %GetControl computes the next control [v, w]' to apply, it is 
            % the composition between the feedforward term and the feedback
            % term computed by MPC on the linearized system.
            %input:  -p_r:[coloum vector 2x1] desired position
            %        -d_r:[coloum vector 2x1] desired velocities
            %        -dd_r:[coloum vector 2x1] desired acceleration 
            %        -direction:[string] "forward" or "backward", direction of motion
            %        -Q:[matrix 2x2] state cost matrix for MPC feedback
            %        -R:[matrix 2x2] input cost matrix for MPC feedback
            %        -sampling_time: sampling time in order to compute A
            %        and B of the discretized system via forward Euler
            %        -prediction_horizon: prection horizon of MPC, the
            %        control horizon is chosen equal the prediction horizon
            %output: -u:[coloum vector 2x1] the control input to be applied
            %        -sym:[boolean] debug utily, return if the hessian is symmetric
            
            [v_r, w_r] = feedforwardControl(p_r, d_r, dd_r, direction);
            %state error
            z_e_k = [p_r - obj.q(1:2); d_r - obj.dq(1:2)];
            [v_f, w_f, obj.xi, sym] = feedbackControl(obj.A, obj.B, Q, R, obj.input_constaint, ...
                                                      z_e_k, obj.q(3), obj.xi, sampling_time, obj.n, obj.m, prediction_horizon);  
            
            u = [v_r + v_f; w_r + w_f];
            %disp(["v_f: ", v_f,"w_f: ",w_f ]);
        end
        
        function state = GetState(obj)
            state = [obj.q; obj.dq];
        end
        
        function [v_r, v_l] = GetWheelsVel(obj, input)
            %GetWheelsVel computes the velocities of the wheels in order to
            % obtain the linear and angular velocities given as input
            %input:  -input:[coloum vector 2x1] linear and angular velocities
            %output: -v_r, v_l: velocities of the wheels
            % [THIS METHOD REQUIRES h]
            
            if obj.h == 0
                error('Error. h must be initialized.')
            end
            v_r = input(1) + obj.h * input(2);
            v_l = input(1) - obj.h * input(2);
        end
        
    end
    
    methods(Access = private)
        
        function G = GetG(obj)
            %GetG computes the base of the null space of the constraint
            % matrix A(q) for a given configuration of the robot.
            %input:
            %output: -G:[matrix 3x2] input vector fields computed in the
            %           current configuration
            
            G = [ cos(obj.q(3)) 0;
                  sin(obj.q(3)) 0;
                              0 1];
        end
        
    end
    
end

