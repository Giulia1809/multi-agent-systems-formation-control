clear;
clc;

tuning = true;

Q1_val = [0.001, 0.005, 0.003, 0.01, 0.05, 0.1, 0.5, 1, 2, 4, 5, 10, 20];
Q2_val = [0.001, 0.005, 0.003, 0.01, 0.05, 0.1, 0.5, 1, 2, 4, 5, 10, 20];
R_val  = [0.001, 0.005, 0.003, 0.01, 0.05, 0.1, 0.5, 1, 2, 4, 5, 10, 20];

best_q1 = 0;
best_q2 = 0;
best_r  = 0; 
best_pos_er = Inf;

%magic grid search for tuning. it's not the best approach but the easiest and
%faster (to write of course) one
cycles = 1;
for i=1:size(Q1_val, 2)
    Q = eye(4,4);
    Q(1:2,1:2) = Q(1:2,1:2)*Q1_val(i);
    for j=1:size(Q2_val, 2)
        Q(3:4,3:4) = eye(2,2);
        Q(3:4,3:4) = Q(3:4,3:4)*Q2_val(j);
        for k=1:size(R_val, 2)
            R = eye(2,2);
            R(1:2,1:2) = R(1:2,1:2)*R_val(k);
%             test_single_robot_tracking;
            test_triangular_formation;
            disp(["END OF EXPERIMENT #", cycles, "OF ", ...
                 size(Q1_val, 2)*size(Q2_val, 2)*size(R_val, 2)]);
            drawnow('update')
            if(best_pos_er > total_error)
                best_pos_er = total_error;
                best_q1 = Q1_val(i);
                best_q2 = Q2_val(j);
                best_r = R_val(k);
            end
            cycles = cycles+1;
        end
    end
end

disp(["THE END ", best_pos_er, best_q1, best_q2, best_r]);

