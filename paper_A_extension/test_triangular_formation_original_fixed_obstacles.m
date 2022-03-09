%Implementation of the first esperiment reported in the paper "Decentralized Leader-Follower Formation Control with Obstacle Avoidance of Multiple Unicycle Mobile Robots" by M. Kamel and Y. Zhang. 

if ~exist("tuning",'var') > 0
    clear;
    clc;
    addpath plot_utilities/
end

%obstacles list
obstacle_pos = {[20.0, 2.0], [30.0, -2.0], [5.5, 5.5], [5.5, 3.5], [10.5, -2.5], [10.5, -4.5], [14.6, 6.5], [25, 6.5], [3.5, -6.5], [22.0, -6.5] }; %cell array

%leader initial position
q_s = [1, 0, 0]';
leader = Unicycle(q_s);

%sampling time and duration of experiment
step = 0.05;
T = 80;

%initialization of trajectory
f1_des = [-5, -4];
f2_des = [-5, 4];

leader_traj = trajectoryLinear(1, 40, 80);
follower1_traj = @(state_l) [state_l(1:2)' + f1_des;
                            state_l(4:5)';
                            0, 0;];
follower2_traj = @(state_l) [state_l(1:2)' + f2_des;	
                            state_l(4:5)';
                            0, 0;];

%followers initial positions
q_s_f1 = [0, -6, 90]';
q_s_f2 = [0, 2, 90]';
follower_1 = Unicycle(q_s_f1);
follower_2 = Unicycle(q_s_f2);

total_error = 0;

%parameters of controller
p = 5;                   %prediction horizon

%optimal params for leader
Q_leader = 0.01 * eye(4,4);   %state cost matrix
Q_leader(3,3) = 0.05;
Q_leader(4,4) = 0.05;
R_leader = 20*eye(2,2);

%optimal params for follower 1
Q_1 = 0.04541 * eye(4,4);   %state cost matrix
Q_1(3,3) = 0.010;
Q_1(4,4) = 0.010;
R_1 = 20*eye(2,2);

Q_2 = 0.049 * eye(4,4);   %state cost matrix
Q_2(3,3) = 0.0105;
Q_2(4,4) = 0.0105;
R_2 = 20*eye(2,2);

if ~exist("tuning",'var')
    Q = eye(4,4);   %state cost matrix
    Q(3,3) = 0.8;
    Q(4,4) = 0.8;
    R = 0.1*eye(2,2);   %input cost matrix

    traj_l = {leader.q(1:2)};
    traj_f1 = {follower_1.q(1:2)};
    traj_f2 = {follower_2.q(1:2)};
    traj_ul = {[0; 0]};
    traj_u1 = {[0; 0]};
    traj_u2 = {[0; 0]};
    
    traj_l_des_in = leader_traj(0); 
    traj_f1_des_in = follower1_traj(leader.GetState()); 
    traj_f2_des_in = follower2_traj(leader.GetState()); 
    traj_l_des = {traj_l_des_in(1,:)'};
    traj_f1_des = {traj_f1_des_in(1,:)'};
    traj_f2_des = {traj_f2_des_in(1,:)'};
    
    err_follower_1 = [follower_1.q(1:2) - traj_f1_des_in(1, :)'];
    err_follower_2 = [follower_2.q(1:2) - traj_f2_des_in(1, :)'];
    
    time_traj = {0};
else
    total_error = 0;
end

vel_des1_k = [0,0]';
vel_des2_k = [0,0]';

for elapse_time = step:step:T
    
    if ~exist("tuning",'var')
        disp(["elapse time: ", elapse_time]);
    end
    
    %get next point of trajectory
    leader_traj_des = leader_traj(elapse_time);
    
    %compute the control
    [leader_u, sym_l] = leader.GetControl(leader_traj_des(1,:)', leader_traj_des(2,:)', ...
                                 leader_traj_des(3,:)', "forward", Q_leader, R_leader, step, p);
                             
    %get obstacles correction
    [c_ul] = obstacleCorrection_fixed(leader.q, obstacle_pos);
    
    leader.Step(leader_u + c_ul, step);
    
    follower1_traj_des = follower1_traj(leader.GetState());
    follower2_traj_des = follower2_traj(leader.GetState());
    
    acc_des1 = (follower1_traj_des(2,:)' - vel_des1_k) / step;
    acc_des2 = (follower2_traj_des(2,:)' - vel_des2_k) / step;
    
    vel_des1_k = follower1_traj_des(2,:)';
    vel_des2_k = follower2_traj_des(2,:)';
    
    [follower1_u, sym_1] = follower_1.GetControl(follower1_traj_des(1,:)', follower1_traj_des(2,:)', ...
                                                 acc_des1, "forward", Q_1, R_1, step, p);
    [follower2_u, sym_2] = follower_2.GetControl(follower2_traj_des(1,:)', follower2_traj_des(2,:)', ...
                                                 acc_des2, "forward", Q_2, R_2, step, p);
    
    if ~sym_l && ~sym_1 && ~sym_2 && exist("tuning",'var')
        total_error = Inf;
        break;
    end

    %get obstacles correction
    [c_u1] = obstacleCorrection_fixed(follower_1.q, obstacle_pos);
    [c_u2] = obstacleCorrection_fixed(follower_2.q, obstacle_pos);
            
    follower_1.Step(follower1_u + c_u1, step);
    follower_2.Step(follower2_u + c_u2, step);
    
    if ~exist("tuning",'var')
        traj_l{end+1} = leader.q(1:2);
        traj_f1{end+1} = follower_1.q(1:2);
        traj_f2{end+1} = follower_2.q(1:2);
        traj_ul{end+1} = leader_u + c_ul;
        traj_u1{end+1} = follower1_u + c_u1;
        traj_u2{end+1} = follower2_u + c_u2;
        
        traj_l_des{end+1} = leader_traj_des(1,:)';
        traj_f1_des{end+1} = follower1_traj_des(1,:)';
        traj_f2_des{end+1} = follower2_traj_des(1,:)';
        
        err_follower_1 = [err_follower_1, follower_1.q(1:2) - follower1_traj_des(1, :)'];
        err_follower_2 = [err_follower_2, follower_2.q(1:2) - follower2_traj_des(1, :)'];
    
        time_traj{end+1} = elapse_time;
        
    else
        
        pos_err_l = (leader_traj_des(1,:)' - leader.q(1:2))' * (leader_traj_des(1,:)' - leader.q(1:2));                 
        pos_err_1 = (follower1_traj_des(1,:)' - follower_1.q(1:2))' * (follower1_traj_des(1,:)' - follower_1.q(1:2));   
        pos_err_2 = (follower2_traj_des(1,:)' - follower_2.q(1:2))' * (follower2_traj_des(1,:)' - follower_2.q(1:2));   
        
        for ind = 1:size(obstacle_pos, 2)
            obs = obstacle_pos{ind};
            d = norm(obs' - follower_1.q(1:2));
            if d <= 1
                pos_err_1 = pos_err_1 + 10 * ( 1 / d);
            end
        end

        total_error = total_error + pos_err_1;% + pos_err_l + pos_err_2;
    
    end
end

if exist("tuning",'var')
    return
end

%% plots 
total_traj_des = {traj_l_des traj_f1_des traj_f2_des};
total_traj = {traj_l traj_f1 traj_f2};
u_total_traj = {traj_ul traj_u1 traj_u2};
color = {'blue', 'red', '#66CC00'};

%trajectory 
clf;
grid minor;
xlim([-5, 45]);
for ind = 1:size(obstacle_pos, 2)
        obs = obstacle_pos{ind};
        circles(obs(1), obs(2), 0.5, 'facecolor', 'black', 'HandleVisibility','off');
        text(obs(1)-2.5, obs(2),num2str(ind),'Color','#990099');
end
circles(q_s(1), q_s(2), .5, 'facecolor', 'blue', 'HandleVisibility','off');
circles(q_s_f1(1), q_s_f1(2), .5, 'facecolor', 'red', 'HandleVisibility','off');
circles(q_s_f2(1), q_s_f2(2), .5, 'facecolor', '#66CC00','HandleVisibility','off');
for i=1:size(total_traj, 2)
    matrix = cell2mat(total_traj{i});
    x = matrix(1, :);
    y = matrix(2, :);
    line(x, y, 'Color', color{i}, 'LineWidth', 1);
end
legend(["Leader", "Follower 1", "Follower 2"], 'location','best');
circles(leader.q(1), leader.q(2), .5, 'facecolor', 'blue', 'HandleVisibility','off');
circles(follower_1.q(1), follower_1.q(2), .5, 'facecolor', 'red', 'HandleVisibility','off');
circles(follower_2.q(1), follower_2.q(2), .5, 'facecolor', '#66CC00', 'HandleVisibility','off');
ylabel('y [m]');
xlabel('x [m]');
saveas(gcf, 'plots_original_fixed/trajectories.png');

pause;

%trajectory velocities
%trajectory v
clf;
subplot(2, 1, 1);
ylim([0, 1.25]);
yticks(0:.25:1.25);
title("Linear Velocities of the Robots");
grid minor;
for i=1:size(u_total_traj, 2)
    matrix = cell2mat(u_total_traj{i});
    y = matrix(1, :);
    x = cell2mat(time_traj);
    line(x, y, 'Color', color{i}, 'LineWidth', 1);
end
legend(["Leader", "Follower 1", "Follower 2"], 'location','best');
ylabel('v [m/s]');
xlabel('t [s]');

%trajectory w
subplot(2, 1, 2);
ylim([-2, 11]);
yticks(-2:2:10);
title("Angular Velocities of the Robots");
grid minor;
for i=1:size(u_total_traj, 2)
    matrix = cell2mat(u_total_traj{i});
    y = matrix(2, :);
    x = cell2mat(time_traj);
    line(x, y, 'Color', color{i}, 'LineWidth', 1);
end
legend(["Leader", "Follower 1", "Follower 2"], 'location','best');
ylabel('w [rad/s]');
xlabel('t [s]');
saveas(gcf, 'plots_original_fixed/v_traj.png');

pause;

%trajectory errors
%first follower
clf;
subplot(2, 1, 1);
title("Errors in the Desired Formation of the First Follower");
grid minor;
x = cell2mat(time_traj);
for i=1:2
    y = err_follower_1(i, :);
    line(x, y, 'Color', color{i}, 'LineWidth', 1);
end
legend(["x-axis", "y-axis"], 'location','best');
ylabel('errors in x,y [m]');
xlabel('t [s]');

%second follower
subplot(2, 1, 2);
title("Errors in the Desired Formation of the Second Follower");
grid minor;
for i=1:2
    y = err_follower_2(i, :);
    line(x, y, 'Color', color{i}, 'LineWidth', 1);
end
legend(["x-axis", "y-axis"], 'location','best');
ylabel('errors in x,y [m]');
xlabel('t [s]');
saveas(gcf, 'plots_original_fixed/errors.png');
