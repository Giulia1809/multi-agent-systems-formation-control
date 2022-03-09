classdef VRepBridge < matlab.mixin.Copyable
% Bridge between Coppelisim scene and Matlab 
    properties
        leader_q;           %[x, y, phi] generalized coordinates of leader robot
        leader_dq;          %[dx, dy, dphi] time derivatives of leader robot

        follower1_q;        %[x, y, phi] generalized coordinates of follower1 robot
        follower1_dq;       %[dx, dy, dphi] time derivatives of follower1 robot

        follower2_q;        %[x, y, phi] generalized coordinates of follower2 robot
        follower2_dq;       %[dx, dy, dphi] time derivatives of follower2 robot

        leader_v_r;         % leader right wheel velocity
        leader_v_l;         % leader left wheel velocity
        follower1_v_r;      % follower1 right wheel velocity
        follower1_v_l;      % follower1 left wheel velocity
        follower2_v_r;      % follower2 right wheel velocity
        follower2_v_l;      % follower2 left wheel velocity
    end

    methods
   
        function [sim, clientID] =  intilizeRemoteAPI(obj)
            sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
            sim.simxFinish(-1); % just in case, close all opened connections
            clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
            disp("Remote API Initialized")
        end

        function obj = getRobotState(obj, sim, clientID)
            %Get the state (position and orientation) of all the robots
            
            if (clientID>-1)
                [returnCode, obj.leader_q(1)] = sim.simxGetFloatSignal(clientID, 'FeedbackPose/leader/position/x', sim.simx_opmode_streaming);
                [returnCode, obj.leader_q(2)] = sim.simxGetFloatSignal(clientID, 'FeedbackPose/leader/position/y', sim.simx_opmode_streaming);
                [returnCode, obj.leader_q(3)] = sim.simxGetFloatSignal(clientID, 'FeedbackPose/leader/orientation/psi', sim.simx_opmode_streaming);
                [returnCode, obj.leader_dq(1)] = sim.simxGetFloatSignal(clientID, 'FeedbackPose/leader/linear_velocity/x', sim.simx_opmode_streaming);
                [returnCode, obj.leader_dq(2)] = sim.simxGetFloatSignal(clientID, 'FeedbackPose/leader/linear_velocity/y', sim.simx_opmode_streaming);
                [returnCode, obj.leader_dq(3)] = sim.simxGetFloatSignal(clientID, 'FeedbackPose/leader/angular_velocity/z', sim.simx_opmode_streaming);

                [returnCode, obj.follower1_q(1)] = sim.simxGetFloatSignal(clientID, 'FeedbackPose/follower1/position/x', sim.simx_opmode_streaming);
                [returnCode, obj.follower1_q(2)] = sim.simxGetFloatSignal(clientID, 'FeedbackPose/follower1/position/y', sim.simx_opmode_streaming);
                [returnCode, obj.follower1_q(3)] = sim.simxGetFloatSignal(clientID, 'FeedbackPose/follower1/orientation/psi', sim.simx_opmode_streaming);
                [returnCode, obj.follower1_dq(1)] = sim.simxGetFloatSignal(clientID, 'FeedbackPose/follower1/linear_velocity/x', sim.simx_opmode_streaming);
                [returnCode, obj.follower1_dq(2)] = sim.simxGetFloatSignal(clientID, 'FeedbackPose/follower1/linear_velocity/y', sim.simx_opmode_streaming);
                [returnCode, obj.follower1_dq(3)] = sim.simxGetFloatSignal(clientID, 'FeedbackPose/follower1/angular_velocity/z', sim.simx_opmode_streaming);

                [returnCode, obj.follower2_q(1)] = sim.simxGetFloatSignal(clientID, 'FeedbackPose/follower2/position/x', sim.simx_opmode_streaming);
                [returnCode, obj.follower2_q(2)] = sim.simxGetFloatSignal(clientID, 'FeedbackPose/follower2/position/y', sim.simx_opmode_streaming);
                [returnCode, obj.follower2_q(3)] = sim.simxGetFloatSignal(clientID, 'FeedbackPose/follower2/orientation/psi', sim.simx_opmode_streaming);
                [returnCode, obj.follower2_dq(1)] = sim.simxGetFloatSignal(clientID, 'FeedbackPose/follower2/linear_velocity/x', sim.simx_opmode_streaming);
                [returnCode, obj.follower2_dq(2)] = sim.simxGetFloatSignal(clientID, 'FeedbackPose/follower2/linear_velocity/y', sim.simx_opmode_streaming);
                [returnCode, obj.follower2_dq(3)] = sim.simxGetFloatSignal(clientID, 'FeedbackPose/follower2/angular_velocity/z', sim.simx_opmode_streaming);
            else
                disp('Failed connecting to remote API server');
            end
        end

        function obj = sendToVrep(obj, sim, clientID)
            % Send the wheel velocities of all the robots

            if (clientID>-1)
                disp('Connected to remote API server');

                [returnCode] = sim.simxSetFloatSignal(clientID, 'EstimatedPose/leader/right_vel', obj.leader_v_r, sim.simx_opmode_oneshot);
                [returnCode] = sim.simxSetFloatSignal(clientID, 'EstimatedPose/leader/left_vel', obj.leader_v_l, sim.simx_opmode_oneshot);
                [returnCode] = sim.simxSetFloatSignal(clientID, 'EstimatedPose/follower1/right_vel', obj.follower1_v_r, sim.simx_opmode_oneshot);
                [returnCode] = sim.simxSetFloatSignal(clientID, 'EstimatedPose/follower1/left_vel', obj.follower1_v_l, sim.simx_opmode_oneshot);
                [returnCode] = sim.simxSetFloatSignal(clientID, 'EstimatedPose/follower2/right_vel', obj.follower2_v_r, sim.simx_opmode_oneshot);
                [returnCode] = sim.simxSetFloatSignal(clientID, 'EstimatedPose/follower2/left_vel', obj.follower2_v_l, sim.simx_opmode_oneshot);

                % Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
                sim.simxGetPingTime(clientID);

                % Now close the connection to CoppeliaSim:    
                sim.simxFinish(clientID);
            else
                disp('Failed connecting to remote API server');
            end
        end

        function distructor(obj, sim, clientID)
            % Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
            sim.simxGetPingTime(clientID);

            % Now close the connection to CoppeliaSim:    
            sim.simxFinish(clientID);
            
            sim.delete(); % call the destructor!

            disp('VREP Conntection Closed');
        end
    
    end
end
