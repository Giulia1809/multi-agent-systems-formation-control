function trajectory = followerTrajectory(l_d, phi_d)
%TODO: comment
    
    lx = l_d * cos(phi_d);
    ly = l_d * sin(phi_d);
    
    trajectory = @(ls) [                 ls(1) + lx * cos(ls(3)) - ly * sin(ls(3)),                 ls(2) + lx * sin(ls(3)) + ly * cos(ls(3));
                         ls(4) - lx * sin(ls(3)) * ls(6) - ly * cos(ls(3)) * ls(6), ls(5) + lx * cos(ls(3)) * ls(6) - ly * sin(ls(3)) * ls(6);
                         0, 0];%-lx*cos(ls(3))*ls(6)^2 + ly*sin(ls(3))*ls(6)^2, -lx*sin(ls(3))*ls(6)^2 - ly*cos(ls(3))*ls(6)];
end

