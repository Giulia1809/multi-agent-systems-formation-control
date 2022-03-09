%Single robot tracking an 8-shhape trajectory

if ~exist("tuning",'var') > 0
    clear;
    clc;
end

% % Create and Initialize Remote API Connection
% vrep_bridge = VRepBridge();
% [sim, clientID] = vrep_bridge.intilizeRemoteAPI();

%robot initial position
q_s = [1, 0.78, deg2rad(45)]';
robot = Unicycle(q_s);

%sampling time and duration of experiment
step = 0.05;
T = 30;

%initialization of trajectory
traj_fun = trajectory8Shaped();

%parameters of controller
p = 5;          %prediction horizon

if ~exist("tuning",'var')
    Q = 0.05 * eye(4,4);   %state cost matrix
    Q(3,3) = 0.005;
    Q(4,4) = 0.005;
    R = 20*eye(2,2);   %input cost matrix
    traj_des_in = traj_fun(0);
    total_traj = {robot.q(1:2)};
    total_traj_des = {traj_des_in(1, :)'};
else
    total_error = 0;
end

for elapse_time = step:step:T
    
    if ~exist("tuning",'var')
        disp(["elapse time: ", elapse_time]);
    end
    
    %get next point of trajectory
    traj_des = traj_fun(elapse_time);
    
    %compute the control
    [u, sym] = robot.GetControl(traj_des(1,:)', traj_des(2,:)', traj_des(3,:)', "forward", Q, R, step, p);
    
    if ~sym && exist("tuning",'var')
        total_error = Inf;
        break;
    end
    
    %feed the new input
    robot.Step(u, step);
    
    %compute the norm of the position error
    pos_err = traj_des(1,:)' - robot.q(1:2);
    
    if ~exist("tuning",'var')
        disp(["norm position error: ", pos_err' * pos_err]);
        
        total_traj{end+1} = robot.q(1:2);
        total_traj_des{end+1} = traj_des(1,:)';
    else
        total_error = total_error + pos_err' * pos_err;
    end
    
end

% % Call Remote API destructor to Close the connection
% vrep_bridge.distructor(sim, clientID);

%% plots 

if ~exist("tuning",'var')
    clf;
    xlim([0, 2]);
    ylim([0, 2]);
    grid minor;
    matrix = cell2mat(total_traj_des);
    x = matrix(1, :);
    y = matrix(2, :);
    line(x, y, 'Color', 'blue' , 'LineWidth', 1);
    matrix1 = cell2mat(total_traj);
    x = matrix1(1, :);
    y = matrix1(2, :);
    line(x, y, 'Color', 'red' , 'LineWidth', 1);
    ylabel('y [m]');
    xlabel('x [m]');
    saveas(gcf, 'plots/single_robot_trajectory_tracking.png');
end