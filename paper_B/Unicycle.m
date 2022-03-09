classdef Unicycle < matlab.mixin.Copyable
    %kinematic model of the differential driver robot
    
    properties
        q;      %[x, y, theta] generalized coordinates
        dq;     %[dx, dy, dtheta] time derivatives 
        t;      %time
        h;      %distance between the vehicle longitudinal axis and each wheel
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
        
        function state = GetState(obj)
            state = [obj.q; obj.dq];
        end
        
        function [v_r, v_l] = GetWheelsVel(obj, input)
            %GetWheelsVel computes the velocities of the wheels in order to
            %           obtain the linear and angular velocities given as input
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
            %GetG computes the input vector fields for a given configuration 
            %           of the robot.
            %input:
            %output: -G:[matrix 3x2] input vector fields computed in the
            %           current configuration
            
            G = [ cos(obj.q(3)) 0;
                  sin(obj.q(3)) 0; 
                              0 1];
        end
        
    end
    
end

