function trajectory = trajectoryLinear(x0, x1, T)
	%trajectoryLinear returns a trajectory on an horizontal line
	%input:  -x0: initial x value 
    %        -x1: final x value
	%		 -T : total time
    %output: -trajectory:[function] trajectory as function of t 
    
    trajectory = @(t) [x0 + (x1-x0) * (t/T), 0.0;
                                  (x1-x0)/T, 0.0;
                                        0.0, 0.0];
%                         [x0 + (x1-x0) * (t^3/T^3) * ( 6*(t^2/T^2) - 15*(t/T) + 10 ),  0;
%                        -(30*t^2*(T - t)^2*(x0 - x1))/T^5,                           0;
%                        -(60*t*(x0 - x1)*(T^2 - 3*T*t + 2*t^2))/T^5,                 0];  %Quintic polynomial
end

