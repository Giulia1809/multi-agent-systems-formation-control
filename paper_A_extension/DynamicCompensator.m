classdef DynamicCompensator < matlab.mixin.Copyable
    %Input-output dynamic feedback linearization 
    % of the differential drive robot
    
    properties
        system; %object of the class Unicycle
        xi;     %internal state of the compensator (linear velocity v)
        z;      %output of the linearized system
        dz;     %derivative of the output
    end
    
    methods
        function obj = DynamicCompensator(system, xi)
            %Construct an instance of this class
            %input:  -system:[Unicycle] system to linearize
            %        -xi:[scalar]initialization of the compensator state
            %output: -obj:[DynamicCompensator] an instance of this class
            
            obj.system = system;
            obj.xi = xi;
            obj.z = system.q(1:2);
            obj.dz = [obj.xi * cos(system.q(3)); obj.xi * sin(system.q(3))]; 
        end
        
        function obj = Step(obj, u, delta_t)
            %Step performs the integration of the linearized unicycle
            % kinematic model using Euler integration
            %input:  -u:[coloum vector 2x1] new input of the system
            %        -delta_t:[scalar] time interval for integration
            %output: -obj:[SIDE EFFECT] this method update q and dq of the
            %             unicycle and the state of the compensator
            
            c = obj.GetC * u;
            a = c(1);
            obj.xi = a * delta_t;
            w = c(2);
            %disp("linear vel: "+obj.xi+" angular vel: "+w); DEBUG
            obj.system.Step([obj.xi; w], delta_t);
            obj.z = obj.system.q(1:2);
            obj.dz = [obj.xi * cos(obj.system.q(3)); obj.xi * sin(obj.system.q(3))];
        end
    end
    
    methods(Access = private)
        
        function C = GetC(obj)
            %GetC computes inverse of the differential map 
            %input:
            %output: -C:[matrix 2x2] input matrix transformation
            
            C = 1/obj.xi * [obj.xi * cos(obj.system.q(3)), obj.xi * sin(obj.system.q(3));
                                    -sin(obj.system.q(3)),          sin(obj.system.q(3))];
        end
        
    end
end

