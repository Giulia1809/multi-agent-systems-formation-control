function trajectory = trajectoryEllipticalv2(r_x, r_y, dim)
%TODO: comment
    
    trajectory = @(t) [       dim*cos(t*r_x),         dim*sin(t*r_y);
                         -dim*sin(t*r_x)*r_x,     dim*cos(t*r_y)*r_y;
                       -dim*cos(t*r_x)*r_x^2, -dim*sin(t*r_y)*r_y^2];
end

