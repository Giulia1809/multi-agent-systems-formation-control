%Single robot tracking an 8-shhape trajectory
clear;
clc;

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
%robot initial position
robot = VrepUnicycle(sim, clientID, vrep_step, 'leader', h, R);

%sampling time and duration of experiment
T = 30;

%initialization of trajectory
traj_fun = trajectory8Shaped();

%parameters of controller
p = 5;          %prediction horizon

Q = 3 * eye(4,4);   %state cost matrix
Q(3,3) = 0.001;
Q(4,4) = 0.001;
R = 5*eye(2,2);   %input cost matrix

for elapse_time = vrep_step:vrep_step:T
    
    disp(["elapse time: ", elapse_time]);
    
    %get next point of trajectory
    traj_des = traj_fun(elapse_time);
    
    %compute the control
    [u, sym] = robot.GetControl(traj_des(1,:)', traj_des(2,:)', traj_des(3,:)', "forward", Q, R, vrep_step, p);
    
    %feed the new input
    robot.Step(u, vrep_step, false);
    
    %compute the norm of the position error
    pos_err = traj_des(1,:)' - robot.q(1:2);
    disp(["norm position error: ", pos_err' * pos_err]);  
end

sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
sim.simxFinish(-1);
sim.delete();
disp('VREP Conntection Closed');

