% 3 robots mantain a formation during the execution of a trajectory
clear;
clc;

addpath plot_utilities/
addpath vrep_libraries/

port = 19999;
vrep_step = 0.05;
sim = remApi('remoteApi');			%RemoteAPI object
sim.simxFinish(-1);
clientID = sim.simxStart('127.0.0.1', port, true, true, 5000, 5);
if (clientID > -1)
    disp('Connected to simulator');
else
    error('Error in connection');
end
% enable the synchronous mode on the client: (integration step on call)
sim.simxSynchronous(clientID, true);
% start the simulation:
sim.simxStartSimulation(clientID, sim.simx_opmode_blocking);

h = 0.32;
R = 0.095;

leader = VrepUnicycle(sim, clientID, vrep_step, 'leader', h, R);

%sampling time and duration of experiment
T = 24;

%initialization of trajectory
l1_des = 0.5;
phi1_des = deg2rad(135);
l2_des = 0.5;
phi2_des = deg2rad(225);

leader_traj = trajectoryElliptical(0.25, 0.25);
follower1_traj = followerTrajectory(l1_des, phi1_des);
follower2_traj = followerTrajectory(l2_des, phi2_des);

%followers initial positions
% traj_f1_des_in = follower1_traj(leader.GetState()); 
% traj_f2_des_in = follower2_traj(leader.GetState()); 
% 
% q_s_f1 = [traj_f1_des_in(1,:), deg2rad(100)]';
% q_s_f2 = [traj_f2_des_in(1,:), deg2rad(100)]';
follower_1 = VrepUnicycle(sim, clientID, vrep_step, 'follower1', h, R);
follower_2 = VrepUnicycle(sim, clientID, vrep_step, 'follower2', h, R);

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

vel_des1_k = [0,0]';
vel_des2_k = [0,0]';

% uncomment here for fix strange behavoiur of vrep
% for i=1:2000
%     leader.UpdateState();
%     follower_1.UpdateState();
%     follower_2.UpdateState();
% end

for elapse_time = vrep_step:vrep_step:T
    
    disp(["elapse time: ", elapse_time]);
    
    %get next point of trajectory
    leader_traj_des = leader_traj(elapse_time);
    
    %compute the control
    [leader_u, sym_l] = leader.GetControl(leader_traj_des(1,:)', leader_traj_des(2,:)', ...
                                 leader_traj_des(3,:)', "forward", Q_leader, R_leader, vrep_step, p);
                             
    leader.Step(leader_u, vrep_step, true);
    
    
    follower1_traj_des = follower1_traj(leader.GetState());
    follower2_traj_des = follower2_traj(leader.GetState());
    
    acc_des1 = (follower1_traj_des(2,:)' - vel_des1_k) / vrep_step;
    acc_des2 = (follower2_traj_des(2,:)' - vel_des2_k) / vrep_step;
    
    vel_des1_k = follower1_traj_des(2,:)';
    vel_des2_k = follower2_traj_des(2,:)';
    
    [follower1_u, sym_1] = follower_1.GetControl(follower1_traj_des(1,:)', follower1_traj_des(2,:)', ...
                                                 acc_des1, "forward", Q_1, R_1, vrep_step, p);
    [follower2_u, sym_2] = follower_2.GetControl(follower2_traj_des(1,:)', follower2_traj_des(2,:)', ...
                                                 acc_des2, "forward", Q_2, R_2, vrep_step, p);
                                    
    follower_1.Step(follower1_u, vrep_step, true);
    follower_2.Step(follower2_u, vrep_step, false);
    
    leader.UpdateState();
    follower_1.UpdateState();
        
    pos_err_l = (leader_traj_des(1,:)' - leader.q(1:2))' * (leader_traj_des(1,:)' - leader.q(1:2));
    pos_err_1 = (follower1_traj_des(1,:)' - follower_1.q(1:2))' * (follower1_traj_des(1,:)' - follower_1.q(1:2));
    pos_err_2 = (follower2_traj_des(1,:)' - follower_2.q(1:2))' * (follower2_traj_des(1,:)' - follower_2.q(1:2));

    total_error = total_error + pos_err_1 + pos_err_l + pos_err_2;
end

sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
sim.simxFinish(-1);
sim.delete();
disp('VREP Conntection Closed');