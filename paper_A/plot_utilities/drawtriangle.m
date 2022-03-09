function drawtriangle(x_pos,y_pos,theta)
%TODO: REWRITE CLEANER
    w = 1; %width of triangle
    ar = 0.95; % Aspect ratio for equilateral triangle
    h = ar * w;%height of triangle
    rot = [sin(theta), cos(theta);
          -cos(theta), sin(theta)];
    hom = eye(3,3);
    hom(1:2, 1:2) = rot;
    hom(1:2, 3) = [x_pos; y_pos];
    v1 = [0; 0; 1];
    v2 = [w; 0; 1];
    v3 = [w/2; h; 1];
%     x=[0 w w/2];%x coordinates of vertices
%     y=[0 0 h]+[5,5,5];%y coordinates of vertices
    v1_d = hom * v1;
    v2_d = hom * v2;
    v3_d = hom * v3;
    x = [v1_d(1), v2_d(1), v3_d(1)];
    y = [v1_d(2), v2_d(2), v3_d(2)];
    patch(x, y, 'red')
    daspect([1 1 1]);%equal data unit length along x and y axis
    xlim([-5 15])
    ylim([-5 15])
end 