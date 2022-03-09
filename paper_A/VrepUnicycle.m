classdef VrepUnicycle < matlab.mixin.Copyable
    %kinematic model of the differential driver robot
    
    properties
        sim;
        clientId;
        vrep_step;
        name;
        q;      %[x, y, phi] generalized coordinates
        dq;     %[dx, dy, dphi] time derivatives 
        h;      %distance between the vehicle longitudinal axis and each wheel
        R;
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
        function obj = VrepUnicycle(sim, clientId, vrep_step, name, h, R)
            %Construct an instance of this class
            %input:  -h:[scalar] distance between the wheels
            %        -vrep stufs 
            %output: -obj:[Unicycle] an instance of this class
            
            if ~(clientId > -1)
                obj.sim.simxStopSimulation(obj.clientId, obj.sim.simx_opmode_blocking);
                obj.sim.simxFinish(-1);
                obj.sim.delete();
                error('Failed connecting to remote API server')
            end
            
            obj.vrep_step = vrep_step;
            obj.sim = sim;
            obj.clientId = clientId;
            obj.name = name;
            
            obj.q  = zeros(3, 1);
            obj.dq = zeros(3, 1);
            [~, obj.q(1)] = obj.sim.simxGetFloatSignal(obj.clientId, ...
                strcat('FeedbackPose/', obj.name, '/position/x'), obj.sim.simx_opmode_streaming);
            [~, obj.q(2)] = obj.sim.simxGetFloatSignal(obj.clientId, ...
                strcat('FeedbackPose/', obj.name, '/position/y'), obj.sim.simx_opmode_streaming);
            [~, obj.q(3)] = obj.sim.simxGetFloatSignal(obj.clientId, ...
                strcat('FeedbackPose/', obj.name, '/orientation/psi'), obj.sim.simx_opmode_streaming);
            [~, obj.dq(1)] = obj.sim.simxGetFloatSignal(obj.clientId, ...
                strcat('FeedbackPose/', obj.name, '/linear_velocity/x'), obj.sim.simx_opmode_streaming);
            [~, obj.dq(2)] = obj.sim.simxGetFloatSignal(obj.clientId, ...
                strcat('FeedbackPose/', obj.name, '/linear_velocity/y'), obj.sim.simx_opmode_streaming);
            [~, obj.dq(3)] = obj.sim.simxGetFloatSignal(obj.clientId, ...
                strcat('FeedbackPose/', obj.name, '/angular_velocity/z'), obj.sim.simx_opmode_streaming);

            obj.xi = 0;
            
            obj.input_constaint = [-0.5, 0.5;
                                   -0.5, 0.5];
            
            obj.n = 4;
            obj.m = 2;
            obj.A = zeros(obj.n, obj.n);
            obj.A(1:2, 3:4) = eye(2, 2);
            obj.B = zeros(obj.n, obj.m);
            obj.B(3:4, :) = eye(2, 2);
            
            obj.h = h;
            obj.R = R;
        end
        
        function obj = Step(obj, input, delta_t, wait)
            %Step does one step in the vrep environment
            %input:  -input:[coloum vector 2x1] linear and angular velocities
            %        -delta_t:[scalar] time interval for integration
            %output: -obj:[SIDE EFFECT] this method update q and dq of the
            %             object
            
            if ~(obj.clientId > -1)
                obj.sim.simxStopSimulation(obj.clientId, obj.sim.simx_opmode_blocking);
                obj.sim.simxFinish(-1);
                obj.sim.delete();
                error('Failed connecting to remote API server')
            end
            
            %send the input
            [v_r, v_l] = obj.GetWheelsVel(input);
            [~] = obj.sim.simxSetFloatSignal(obj.clientId, ...
                strcat('EstimatedPose/', obj.name, '/right_vel'), v_r, obj.sim.simx_opmode_oneshot);
            [~] = obj.sim.simxSetFloatSignal(obj.clientId, ...
                strcat('EstimatedPose/', obj.name, '/left_vel'), v_l, obj.sim.simx_opmode_oneshot);
            
            %trigger the simulation
            if ~wait
                for i = 1:(delta_t/obj.vrep_step)				%Number of integrations in delta_t
                    obj.sim.simxSynchronousTrigger(obj.clientId);	%Triggering the integration
                    % To overcome delay in values according to (Remote API modus operandi) document  
                end
                obj.sim.simxGetPingTime(obj.clientId);
                obj.UpdateState();
            end
            
        end
        
        function obj = UpdateState(obj)
            %update the state
            if ~(obj.clientId > -1)
                obj.sim.simxStopSimulation(obj.clientId, obj.sim.simx_opmode_blocking);
                obj.sim.simxFinish(-1);
                obj.sim.delete();
                error('Failed connecting to remote API server')
            end
            
            [~, obj.q(1)] = obj.sim.simxGetFloatSignal(obj.clientId, ...
                strcat('FeedbackPose/', obj.name, '/position/x'), obj.sim.simx_opmode_buffer);
            [~, obj.q(2)] = obj.sim.simxGetFloatSignal(obj.clientId, ...
                strcat('FeedbackPose/', obj.name, '/position/y'), obj.sim.simx_opmode_buffer);
            [~, obj.q(3)] = obj.sim.simxGetFloatSignal(obj.clientId, ...
                strcat('FeedbackPose/', obj.name, '/orientation/psi'), obj.sim.simx_opmode_buffer);
            [~, obj.dq(1)] = obj.sim.simxGetFloatSignal(obj.clientId, ...
                strcat('FeedbackPose/', obj.name, '/linear_velocity/x'), obj.sim.simx_opmode_buffer);
            [~, obj.dq(2)] = obj.sim.simxGetFloatSignal(obj.clientId, ...
                strcat('FeedbackPose/', obj.name, '/linear_velocity/y'), obj.sim.simx_opmode_buffer);
            [~, obj.dq(3)] = obj.sim.simxGetFloatSignal(obj.clientId, ...
                strcat('FeedbackPose/', obj.name, '/angular_velocity/z'), obj.sim.simx_opmode_buffer);
        end
        
        function [u, sym ]= GetControl(obj, p_r, d_r, dd_r, direction, Q, R, sampling_time, prediction_horizon)
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
            %v_r = input(1) + obj.h * input(2);
            %v_l = input(1) - obj.h * input(2);
            v_r = (2 * input(1) + input(2)*obj.h)/(2*obj.R);
            v_l = (2 * input(1) - input(2)*obj.h)/(2*obj.R);
        end
        
    end  
    
end

