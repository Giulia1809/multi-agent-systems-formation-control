% 3 robots mantain a formation during the execution of a trajectory
if ~exist("tuning",'var') > 0
    clear;
    clc;
    addpath plot_utilities/
end

% % Create and Initialize Remote API Connection
% vrep_bridge = VRepBridge();
% [sim, clientID] = vrep_bridge.intilizeRemoteAPI();

%leader initial position
q_s = [1.1, -0.1, deg2rad(100)]';
leader = Unicycle(q_s);

%sampling time and duration of experiment
step = 0.05;
T = 24;

%initialization of trajectory
l1_des = 0.5;
phi1_des = deg2rad(135);
l2_des = 0.5;
phi2_des = deg2rad(225);

leader_traj = trajectoryElliptical(0.25, 0.25);
follower1_traj = followerTrajectory(l1_des, phi1_des);
follower2_traj = followerTrajectory(l2_des, phi2_des);

traj_f1_des_in = follower1_traj(leader.GetState()); 
traj_f2_des_in = follower2_traj(leader.GetState()); 

%followers initial positions
q_s_f1 = [traj_f1_des_in(1,:), deg2rad(100)]';
q_s_f2 = [traj_f2_des_in(1,:), deg2rad(100)]';
follower_1 = Unicycle(q_s_f1);
follower_2 = Unicycle(q_s_f2);

total_error = 0;

%parameters of controller
p = 5;                   %prediction horizon

%optimal params for leader
Q_leader = 0.005 * eye(4,4);   %state cost matrix
Q_leader(3,3) = 0.001;
Q_leader(4,4) = 0.001;
R_leader = 20*eye(2,2);

%optimal params for follower 1
Q_1 = 2 * eye(4,4);   %state cost matrix
Q_1(3,3) = 0.001;
Q_1(4,4) = 0.001;
R_1 = 20*eye(2,2);

Q_2 = 2 * eye(4,4);   %state cost matrix
Q_2(3,3) = 0.001;
Q_2(4,4) = 0.001;
R_2 = 20*eye(2,2);

if ~exist("tuning",'var')
    Q = 0.05 * eye(4,4);   %state cost matrix
    Q(3,3) = 0.008;
    Q(4,4) = 0.008;
    R = 20*eye(2,2);   %input cost matrix

    traj_l = {leader.q(1:2)};
    traj_f1 = {follower_1.q(1:2)};
    traj_f2 = {follower_2.q(1:2)};

    traj_l_des_in = leader_traj(0); 
    traj_f1_des_in = follower1_traj(leader.GetState()); 
    traj_f2_des_in = follower2_traj(leader.GetState()); 
    traj_l_des = {traj_l_des_in(1,:)'};
    traj_f1_des = {traj_f1_des_in(1,:)'};
    traj_f2_des = {traj_f2_des_in(1,:)'};

    l1 = {norm(leader.q(1:2) - follower_1.q(1:2))};
    l2 = {norm(leader.q(1:2) - follower_2.q(1:2))};
    dist1 = (follower_1.q(1:2) - leader.q(1:2));
    dist2 = (follower_2.q(1:2) - leader.q(1:2));
    leader_dir = [cos(leader.q(3)), sin(leader.q(3))];
    phi_d1 = {rad2deg(wrapTo2Pi(acos((leader_dir * dist1) / (norm(leader_dir) * norm(dist1)))))};
    phi_d2 = {rad2deg(wrapTo2Pi(-acos((leader_dir * dist2) / (norm(leader_dir) * norm(dist2)))))};
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
                             
    leader.Step(leader_u, step);
    
    
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
                                    
    follower_1.Step(follower1_u, step);
    follower_2.Step(follower2_u, step);
    
    if ~exist("tuning",'var')
        
        l1{end+1} = norm(leader.q(1:2) - follower_1.q(1:2));
        l2{end+1} = norm(leader.q(1:2) - follower_2.q(1:2));
        dist1 = (follower_1.q(1:2) - leader.q(1:2));
        dist2 = (follower_2.q(1:2) - leader.q(1:2));
        leader_dir = [cos(leader.q(3)), sin(leader.q(3))];
        phi_d1{end+1} = rad2deg(wrapTo2Pi(acos((leader_dir * dist1) / (norm(leader_dir) * norm(dist1)))));
        phi_d2{end+1} = rad2deg(wrapTo2Pi(-acos((leader_dir * dist2) / (norm(leader_dir) * norm(dist2)))));
        traj_l_des{end+1} = leader_traj_des(1,:)';
        traj_f1_des{end+1} = follower1_traj_des(1,:)';
        traj_f2_des{end+1} = follower2_traj_des(1,:)';
        traj_l{end+1} = leader.q(1:2);
        traj_f1{end+1} = follower_1.q(1:2);
        traj_f2{end+1} = follower_2.q(1:2);
        time_traj{end+1} = elapse_time;
        
    else
        
        pos_err_l = (leader_traj_des(1,:)' - leader.q(1:2))' * (leader_traj_des(1,:)' - leader.q(1:2));
        pos_err_1 = (follower1_traj_des(1,:)' - follower_1.q(1:2))' * (follower1_traj_des(1,:)' - follower_1.q(1:2));
        pos_err_2 = (follower2_traj_des(1,:)' - follower_2.q(1:2))' * (follower2_traj_des(1,:)' - follower_2.q(1:2));

        total_error = total_error + pos_err_1;% + pos_err_l + pos_err_2;
    
    end
end

% % Call Remote API destructor to Close the connection
% vrep_bridge.distructor(sim, clientID);

if exist("tuning",'var')
    return
end

%% plots 
total_traj_des = {traj_l_des traj_f1_des traj_f2_des};
total_traj = {traj_l traj_f1 traj_f2};
color = {'blue', 'black', 'red'};

%trajectory des
clf;
xlim([-1.5, 1.7]);
ylim([-1.5, 1.7]);
grid minor;
circles(q_s(1), q_s(2), .03, 'facecolor', 'blue');
circles(q_s_f1(1), q_s_f1(2), .03, 'facecolor', 'black');
circles(q_s_f2(1), q_s_f2(2), .03, 'facecolor', 'red');
for i = 1:size(total_traj_des, 2)
    matrix = cell2mat(total_traj_des{i});
    x = matrix(1, :);
    y = matrix(2, :);
    line(x, y, 'Color', color{i}, 'LineWidth', 1);
end
ylabel('y [m]');
xlabel('x [m]');
saveas(gcf, 'plots/trajectories_des.png');

pause;
%trajectory 
clf;
xlim([-1.5, 1.7]);
ylim([-1.5, 1.7]);
grid minor;
circles(q_s(1), q_s(2), .03, 'facecolor', 'blue');
circles(q_s_f1(1), q_s_f1(2), .03, 'facecolor', 'black');
circles(q_s_f2(1), q_s_f2(2), .03, 'facecolor', 'red');
for i=1:size(total_traj, 2)
    matrix = cell2mat(total_traj{i});
    x = matrix(1, :);
    y = matrix(2, :);
    line(x, y, 'Color', color{i}, 'LineWidth', 1);
end
ylabel('y [m]');
xlabel('x [m]');
saveas(gcf, 'plots/trajectories.png');

pause;
%distances
clf;
xlim([0, 24]);
ylim([0.1, 0.6]);
grid minor;
y = cell2mat(l1);
x = cell2mat(time_traj);
grid minor;
line(x, y, 'Color', 'blue', 'LineWidth', 1);

y = cell2mat(l2);
grid minor;
line(x, y, 'Color', 'red', 'LineWidth', 1);
ylabel('Distance [m]');
xlabel('t [s]');

saveas(gca, 'plots/distances.png');

pause;
%angles
clf;
grid minor;
y = cell2mat(phi_d1);
x = cell2mat(time_traj);
grid minor;
line(x, y, 'Color', 'blue', 'LineWidth', 1);

y = cell2mat(phi_d2);
grid minor;
line(x, y, 'Color', 'red', 'LineWidth', 1);
ylabel('Distance [m]');
xlabel('t [s]');

saveas(gca, 'plots/angles.png');