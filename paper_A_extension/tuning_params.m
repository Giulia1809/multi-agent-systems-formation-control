clear;
clc;

tuning = true;

Q1_val = [0.001, 0.005, 0.01, 0.05, 0.1, 0.5, 2, 7];
Q2_val = [0.001, 0.005, 0.01, 0.05, 0.1, 0.5, 2, 7];
R_val  = [1, 2, 5, 10];
d_max_val = [0.5, 1, 2, 2.5, 5];
F_max_val = [1.5, 2, 3, 5];
s_val = [2,3,4];

% Q1_val = [0.005, 0.01, 0.05, 0.1, 0.5, 1, 2.5, 5, 10];
% Q2_val = [0.001, 0.005, 0.01, 0.05, 0.1, 0.5, 1];
% R_val  = [1, 2.5, 5, 10, 15, 20];

best_q1 = 0;
best_q2 = 0;
best_r  = 0; 
best_d = 0;
best_F = 0;
best_s = 0;
best_pos_er = Inf;

best2_q1 = 0;
best2_q2 = 0;
best2_r  = 0;
best2_d = 0;
best2_F = 0;
best2_s = 0;
best2_pos_er = Inf;

best3_q1 = 0;
best3_q2 = 0;
best3_r  = 0;
best3_d = 0;
best3_F = 0;
best3_s = 0;
best3_pos_er = Inf;

%magic grid search for tuning. it's not the best approach but the easiest and
%faster (to write of course) one
cycles = 1;

num_of_ex = size(Q1_val, 2)*size(Q2_val, 2)*size(R_val, 2)*size(d_max_val,2)*size(F_max_val,2)*size(s_val,2);
% num_of_ex = size(Q1_val, 2)*size(Q2_val, 2)*size(R_val, 2);
total_time = 0;
for i=1:size(Q1_val, 2)
    Q = eye(4,4);
    Q(1:2,1:2) = Q(1:2,1:2)*Q1_val(i);
    for j=1:size(Q2_val, 2)
        Q(3:4,3:4) = eye(2,2);
        Q(3:4,3:4) = Q(3:4,3:4)*Q2_val(j);
        for k=1:size(R_val, 2)
            R = eye(2,2);
            R(1:2,1:2) = R(1:2,1:2)*R_val(k);

            for l=1:size(d_max_val,2)
                d_max = d_max_val(l);
                
                for m=1:size(F_max_val,2)
                    F_max = F_max_val(m);
                    
                    for n=1:size(s_val,2) 
                        s = s_val(n);
                
                        tic;
                        try
                            test_ellitical_traj;
                        catch 
                            disp("not good params");
                            continue;
                        end
                        elapsed_time = toc;
                        total_time = total_time + elapsed_time;
                        
                        disp(["END OF EXPERIMENT #", cycles, "OF ", ...
                             num_of_ex]);
                        disp(["ETA: ",(num_of_ex-cycles)*(total_time/cycles), " sec", best_pos_er]);
                        drawnow 
                        
                        if total_error == Inf
                            cycles = cycles+1;
                            continue;
                        end
                        
                        if(best_pos_er > total_error)
                            best3_q1 = best2_q1;
                            best3_q2 = best2_q2;
                            best3_r  = best2_r;
                            best3_d = best2_d;
                            best3_F = best2_F;
                            best3_s = best2_s;
                            best3_pos_er = best2_pos_er;

                            best2_q1 = best_q1;
                            best2_q2 = best_q2;
                            best2_r  = best_r;
                            best2_d = best_d;
                            best2_F = best_F;
                            best2_s = best_s;
                            best2_pos_er = best_pos_er;

                            best_q1 = Q1_val(i);
                            best_q2 = Q2_val(j);
                            best_r = R_val(k);
                            best_d = d_max_val(l);
                            best_F = F_max_val(m);
                            best_s = s_val(n);
                            best_pos_er = total_error;

                        elseif (best2_pos_er > total_error)
                            best3_q1 = best2_q1;
                            best3_q2 = best2_q2;
                            best3_r  = best2_r; 
                            best3_d = best2_d;
                            best3_F = best2_F;
                            best3_s = best2_s;
                            best3_pos_er = best2_pos_er;

                            best2_q1 = Q1_val(i);
                            best2_q2 = Q2_val(j);
                            best2_r = R_val(k);
                            best2_d = d_max_val(l);
                            best2_F = F_max_val(m);
                            best2_s = s_val(n);
                            best2_pos_er = total_error;

                        elseif (best3_pos_er > total_error)
                            best3_q1 = Q1_val(i);
                            best3_q2 = Q2_val(j);
                            best3_r = R_val(k);
                            best3_d = d_max_val(l);
                            best3_F = F_max_val(m);
                            best3_s = s_val(n);
                            best3_pos_er = total_error;
                        end
                        cycles = cycles+1;
                    end
                end
            end 
        end
    end
end

disp("THE END ");
disp(["1st: ", best_pos_er, best_q1, best_q2, best_r, best_d, best_F, best_s]);
disp(["2nd: ", best2_pos_er, best2_q1, best2_q2, best2_r, best2_d, best2_F, best2_s]);
disp(["3rd: ", best3_pos_er, best3_q1, best3_q2, best3_r, best3_d, best3_F, best3_s]);

