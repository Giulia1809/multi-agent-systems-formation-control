function trajectory = trajectoryElliptical(r_x, r_y)
%TODO: comment
    
    trajectory = @(t) [       cos(t*r_x),         sin(t*r_y);
                         -sin(t*r_x)*r_x,     cos(t*r_y)*r_y;
                       -cos(t*r_x)*r_x^2, -sin(t*r_y)*r_y^2];
end

