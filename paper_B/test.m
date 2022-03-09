clear;
clc;

%followers initial positions
q_f1 = [0.2; -0.05; -pi*(3/4)]; %values retrived from plots, not from paper
q_f2 = [0.2; 0.95; -pi/4];      %values retrived from plots, not from paper
%OBS: used reported values as relative, and not global, coordinates
%values from paper (not coherent with plots)
% q_f1 = [0.5, -2.0, pi*(3/4)]';
% q_f2 = [-0.5, -2.0, pi/4]';

follower_1 = Unicycle(q_f1);
follower_2 = Unicycle(q_f2);

follower_1_unconstrained = Unicycle(q_f1);
follower_2_unconstrained = Unicycle(q_f2);
%SEEN FROM INTERNET THAT THESE ROBOTS ARE 235mm WIDTH, CAN BE PUT AS H

%initialization of trajectory
z1_des = [1.7; 0.95];           %values retrived from plots, not from paper          
z2_des = [1.7; -0.05];          %values retrived from plots, not from paper
%values from paper (not coherent with plots)
% z1_des = [-0.5; -0.5];
% z2_des = [0.5; -0.5];

%leader initial position
q_l = [2.2; 0.45; 0];       %values retrived from plots, not reported on paper
%OBS: theta_l non specified but needed for F matrix in (7), chosen such to
%       obtain reference coordinates of followers coherent with plots
u_l = [0; 0];               %OBS: leader is fixed

%sampling time and duration of experiment
delta = 0.1;            %update interval
simulation_time = 35;   %total time of simulation

%parameters of controller
T = 3;              %prediction horizon
Ts = 0.1;  %used in collision avoidance discretization

lambda = 0.3;       %gain parameter in error dynamics (12)
eta = 0.6;          %upperbound in constraint (28)
gamma = 0.25;       %upperbound in constraint (29)

psi = 0.45;         %distance bound in (5), also in constraint (27)
psi_big = 30;       %parameter in distance constraints (6)

d = 0.15;           %offset of controlled position
n = 2;              %total number of followers

R = eye(n);         %gain symmetric positive def matrix in (25)

controller_f1 = FormationControl(T, Ts, lambda, eta, gamma, ...
                psi, psi_big, d, n, 1, R, follower_1.q, z1_des, q_l);
controller_f2 = FormationControl(T, Ts, lambda, eta, gamma, ...
                psi, psi_big, d, n, 2, R, follower_2.q, z2_des, q_l);

controller_f1_unconstrained = FormationControl(T, Ts, lambda, eta, gamma, ...
                psi, psi_big, d, n, 1, R, follower_1_unconstrained.q, z1_des, q_l);
controller_f2_unconstrained = FormationControl(T, Ts, lambda, eta, gamma, ...
                psi, psi_big, d, n, 2, R, follower_2_unconstrained.q, z2_des, q_l);
                
%plot inizialization
time_traj = {0};
traj_f1 = {follower_1.q(1:2)};
traj_f2 = {follower_2.q(1:2)};
w_traj_f2 = {0};
z1 = controller_f1.GetZ(follower_1.q);
z2 = controller_f2.GetZ(follower_2.q);
distance_traj = {norm(z1 - z2)};

traj_f1_unconstrained = {follower_1_unconstrained.q(1:2)};
traj_f2_unconstrained = {follower_2_unconstrained.q(1:2)};
w_traj_f2_unconstrained = {0};

FPRINTF_LENGTH = 0;
fprintf('Elapsed_time: ')
for elapse_time = 0:delta:simulation_time
    
    %print elapsed_time
    for k=1:FPRINTF_LENGTH
        fprintf('\b'); % backspacing previous counter value
    end
    FPRINTF_LENGTH = fprintf("%.2f", elapse_time);
    
    %compute u
    u1 = controller_f1.GetControl(follower_1.q, z1_des, q_l, u_l, ...
                                    controller_f2.e_hat, controller_f2.mu, true);
    u2 = controller_f2.GetControl(follower_2.q, z2_des, q_l, u_l, ...
                                    controller_f1.e_hat, controller_f1.mu, true);

    u1_unconstrained = controller_f1_unconstrained.GetControl(follower_1_unconstrained.q, z1_des, q_l, u_l, ...
                                    controller_f2_unconstrained.e_hat, controller_f2_unconstrained.mu, false);
    u2_unconstrained = controller_f2_unconstrained.GetControl(follower_2_unconstrained.q, z2_des, q_l, u_l, ...
                                    controller_f1_unconstrained.e_hat, controller_f1_unconstrained.mu, false);

    %apply step
    follower_1.Step(u1, delta);
    follower_2.Step(u2, delta);

    follower_1_unconstrained.Step(u1_unconstrained, delta);
    follower_2_unconstrained.Step(u2_unconstrained, delta);

    %updates
    controller_f1.Update(z1_des, q_l, follower_1.q);
    controller_f2.Update(z2_des, q_l, follower_2.q);

    controller_f1_unconstrained.Update(z1_des, q_l, follower_1_unconstrained.q);
    controller_f2_unconstrained.Update(z2_des, q_l, follower_2_unconstrained.q);

    %for plots
    time_traj{end+1} = elapse_time;
    traj_f1{end+1} = follower_1.q(1:2);
    traj_f2{end+1} = follower_2.q(1:2);
    w_traj_f2{end+1} = controller_f2.alpha(2);
    z1 = controller_f1.GetZ(follower_1.q);
    z2 = controller_f2.GetZ(follower_2.q);
    distance_traj{end+1} = norm(z1 - z2); 

    traj_f1_unconstrained{end+1} = follower_1_unconstrained.q(1:2);
    traj_f2_unconstrained{end+1} = follower_2_unconstrained.q(1:2);
    w_traj_f2_unconstrained{end+1} = controller_f2_unconstrained.alpha(2);

end

%% plots   
total_traj = {traj_f1 traj_f2}; 
total_traj_unconstrained = {traj_f1_unconstrained traj_f2_unconstrained}; 
color = {'blue', 'red'};
line_type = {'--', '-'};

close all
desktop = com.mathworks.mde.desk.MLDesktop.getInstance;
myGroup = desktop.addGroup('Plots');
desktop.setGroupDocked('Plots', 0);
myDim   = java.awt.Dimension(2, 2);   % 2 columns, 2 rows
desktop.setDocumentArrangement('Plots', 2, myDim)
figH    = gobjects(1, 4);
bakWarn = warning('off','MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame');

clc

%Follower Plot
figH(1) = figure('WindowStyle', 'docked', 'Name', sprintf('x-y plot of followers'), 'NumberTitle', 'off');
drawnow;
pause(0.02);
set(get(handle(figH(1)), 'javaframe'), 'GroupName', 'Plots');
clf;
daspect([0.99, 0.9, 1]);
xlim([0, 2.5]);
ylim([-0.2, 1.2]);
grid minor;
hold on
for i = 1:size(total_traj, 2)
    matrix = cell2mat(total_traj{i});
    x = matrix(1, :);
    y = matrix(2, :);
    line(x, y, 'Color', color{i}, 'LineWidth', 1);
    plot(x(1), y(1), 'o', 'Color', color{i}, 'LineWidth', 1);
    label = ['   Follower ' num2str(i)];
    text(x(1),y(1),label, 'Color', color{i})
end
plot(z1_des(1),z1_des(2),'X', 'Color', 'blue')
text(z1_des(1),z1_des(2),'   Reference', 'Color', 'blue')
plot(z2_des(1),z2_des(2),'X', 'Color', 'red')
text(z2_des(1),z2_des(2),'   Reference', 'Color', 'red')

plot(q_l(1), q_l(2), 'o', 'Color', 'black', 'LineWidth', 1);
text(q_l(1)-0.1, q_l(2)+0.07, 'Leader', 'Color', 'black')

ylabel('y [m]');
xlabel('x [m]');
saveas(gcf, 'plots/trajectories.png');
hold on

% Followers without input contraints
figH(2) = figure('WindowStyle', 'docked', 'Name', sprintf('x-y plot of followers without input constraints'), 'NumberTitle', 'off');
drawnow;
pause(0.02);
set(get(handle(figH(2)), 'javaframe'), 'GroupName', 'Plots');
clf;
daspect([0.99, 0.9, 1]);
xlim([0, 2.5]);
ylim([-0.2, 1.4]);
grid minor;
hold on
for i = 1:size(total_traj_unconstrained, 2)
    matrix = cell2mat(total_traj_unconstrained{i});
    x = matrix(1, :);
    y = matrix(2, :);
    plot(x, y, line_type{i}, 'Color', color{i}, 'LineWidth', 1);
    plot(x(1), y(1), 'o', 'Color', color{i}, 'LineWidth', 1);
    label = ['   Follower ' num2str(i)];
    text(x(1),y(1),label, 'Color', color{i})
end
plot(z1_des(1),z1_des(2),'X', 'Color', 'blue')
text(z1_des(1),z1_des(2),'   Reference', 'Color', 'blue')
plot(z2_des(1),z2_des(2),'X', 'Color', 'red')
text(z2_des(1),z2_des(2),'   Reference', 'Color', 'red')

plot(q_l(1), q_l(2), 'o', 'Color', 'black', 'LineWidth', 1);
text(q_l(1)-0.1, q_l(2)+0.07, 'Leader', 'Color', 'black')

ylabel('y [m]');
xlabel('x [m]');
saveas(gcf, 'plots/trajectories_unconstrained.png');
hold on

% Maximum Distance Between Followers
figH(3) = figure('WindowStyle', 'docked', 'Name', sprintf('Minimum distace between followers'), 'NumberTitle', 'off');
drawnow;
pause(0.02);
set(get(handle(figH(3)), 'javaframe'), 'GroupName', 'Plots');
clf;
ylim([0, 1.4]);
grid minor;
y = cell2mat(distance_traj);
x = cell2mat(time_traj);
line(x, y, 'Color', 'blue', 'LineWidth', 1);
hold on
plot(x, 0.45*ones(size(x)),'--','Color','red');
text(15.0,0.4,'Lower bound','Color','red');
ylabel('Distance [m]');
xlabel('Time [sec]');
saveas(gcf, 'plots/distance.png');
hold on

% w_i fo the follower 2
figH(4) = figure('WindowStyle', 'docked', 'Name', sprintf('w_i of the follower 2'), 'NumberTitle', 'off');
drawnow;
pause(0.02);
set(get(handle(figH(4)), 'javaframe'), 'GroupName', 'Plots');
clf;
ylim([-4, 2]);
grid minor;
y = cell2mat(w_traj_f2);
y_unconstrained = cell2mat(w_traj_f2_unconstrained);
x = cell2mat(time_traj);
hold on
line(x, y, 'Color', 'blue', 'LineWidth', 1);
text(1.5,0.5,'Constrained MPC','Color','blue');
plot(x, y_unconstrained, '--', 'Color', 'red', 'LineWidth', 1);
text(1.5,-1.5,'Unonstrained MPC','Color','red');
plot(x, 1*ones(size(x)),'--','Color','black');
text(15.0,1.2,'Upper bound','Color','black');
plot(x,-1*ones(size(x)),'--','Color','black');
text(15.0,-1.2,'Lower bound','Color','black');
ylabel('w [rad/s]');
xlabel('Time [sec]');
saveas(gcf, 'plots/w_2.png');
hold on