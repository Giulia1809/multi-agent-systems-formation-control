function trajectory = trajectory8Shaped()
    %trajectory8Shaped returns an anonymous function which for a given
    % instant of time returns a matrix 3x2 which is formed by:
    %   -the position desired [vector 1x2]
    %   -the velocity desired [vector 1x2]
    %   -the acceleration desired [vector 1x2]
    % the obtained trajectory is an 8-shaped trajectory
    
    trajectory = @(t) [       1.1 + 0.7 * sin(2 * pi / 30 * t),               0.9 + 0.7 * sin(4 * pi / 30 * t);
                           (7 * pi * cos((pi * t) / 15)) / 150,         (7 * pi * cos((2 * pi * t) / 15)) / 75;
                       -(7 * pi^2 * sin((pi * t) / 15)) / 2250, - (14 * pi^2 * sin((2 * pi * t) / 15)) / 1125];
end

