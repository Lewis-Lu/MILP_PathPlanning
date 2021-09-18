%% main program for MILP path planning
clear all;
close all;
clc;

% add auxilliary folders
addpath ampl_utils\
addpath data\
addpath mod\
addpath output\

%%
% parameters
pos_init    = [0 0];
vel_init    = [1 0];

wind_disturbance = [0 0];

pos_final   = [8 9 8 9];
vel_final   = [0 0];

disjunction = [4 4];

% waypoints = [0, 1, 5, 6; ...
%              5, 6, 5, 6; ...
%              3, 4, 3, 4; ...
%              0, 1, 1, 2];

opbox       = [-2 10 -2 10]; % [xmin xmax ymin ymax]

num_vel_cst = 8; % circle approximation number for velocity
num_acc_cst = 8; % circle approximation number for acceleration
num_obs_cst = 8;  % circle approximation number for obstacle

acc_max     = 1; % maximum acceleration
vel_max     = 2; % maximum velocity
vel_min     = 1; % minimum velocity

% obstacle parameters
obs_info = [2 2 0.5;
            5 5 0.5];

% time parameters
num_ts      = 50;
dt          = 0.5;

%% MILP for path planning

% data filename
data_name   = 'integer_planner_climber';

% planner callback
output_file = integer_planner_2D_climber(pos_init, vel_init, pos_final, vel_final, disjunction, wind_disturbance, obs_info, opbox, num_vel_cst,...
    num_acc_cst, num_obs_cst, acc_max, vel_max, vel_min, num_ts, dt, data_name);

% analyse the result output file
[flag, ps, vs, as, fs] = analysisSolution(output_file, 'pos', 'vel', 'acc', 'finish');
if flag
    disp('GLPK Extraction finished.')
else
    disp('GLPK Extraction failed.')
end

% should extract where the fs is 1, the time step last till t = [1...fs==1]
finish_idx = find(fs == 1);

zs = 2*ones(1,size(ps(:,1:finish_idx),2));
path_pts = [ps(:,1:finish_idx);zs]';
path{1} = path_pts;


%%
figure;

% plot trajectory
plot(ps(1,1:finish_idx), ps(2,1:finish_idx), 'b-x')

hold on

% draw the disjunction line
op_x = opbox(1):opbox(2);

plot(op_x, ones(1, size(op_x,2)).*disjunction(2), 'k-')
text(0, disjunction(2)-0.5, 'Floor')
text(0, disjunction(2)+0.5, 'Wall')

% draw the initial speed
endptr = pos_init + vel_init.*dt;
initial_speed = [pos_init' endptr'];
plot(initial_speed(1,:), initial_speed(2,:), 'k-', 'LineWidth', 2)

patch([pos_final(1) pos_final(1) pos_final(1) pos_final(1)],[pos_final(2) pos_final(2) pos_final(2) pos_final(2)], 'g')

% wind disturbance
interval = 1;
for i = opbox(3):interval:opbox(4)
    arrow_base = [opbox(1); i];
    quiver(arrow_base(1)+1, arrow_base(2)+1, wind_disturbance(1)/2, wind_disturbance(2)/2,...
        'Color', 'b',...
        'AutoScale', 'on',...
        'MaxHeadSize', 2)
end

% waypoints 
if exist('waypoints', 'var')
    for i = 1:size(waypoints, 1)
        patch([waypoints(i,1) waypoints(i,1) waypoints(i,2) waypoints(i,2)], [waypoints(i,3) waypoints(i,4) waypoints(i,4) waypoints(i,3)], 'y', 'FaceAlpha', .4)
        text(waypoints(i,1), (waypoints(i,3)+waypoints(i,4))/2,  ['WP' num2str(i)])
    end
end

% obstacles
obs_approx = obstacleApproximation(obs_info, num_obs_cst, 1);

for i = 1:size(obs_info, 1)
    obs_centroid = obs_info(i,1:2);
    obs_radius = obs_info(i,3);
    ox = obs_centroid(1) + obs_radius*cos(linspace(0, 2*pi, 100));
    oy = obs_centroid(2) + obs_radius*sin(linspace(0, 2*pi, 100));
    patch(ox, oy, 'r', 'FaceAlpha', .4)
    approxVertice = obs_approx{i};
    patch(approxVertice(1,:), approxVertice(2,:), 'r', 'FaceAlpha', .4)
end

axis equal
grid on
title(['dt = ' num2str(dt) ' s'])
xlim(opbox(1:2))
ylim(opbox(3:4))
xlabel('x (m)')
ylabel('y (m)')

%% To 3D plotting
idx_disjunct = find(ps(1,:) == disjunction(2));
% two surface [xmin xmax ymin ymax]
floor_xy = [opbox(1) opbox(2) opbox(3) disjunction(2)];
wall_xy = [opbox(1) disjunction(2) opbox(3) opbox(4)];

figure;
hold on

plot3(ps(1,1:idx_disjunct), ps(2,1:idx_disjunct), zeros(1,idx_disjunct), 'b-x')
plot3(ps(1,idx_disjunct:finish_idx), ones(1, finish_idx-idx_disjunct+1).*disjunction(2), ps(2,idx_disjunct:finish_idx)-disjunction(2), 'b-x')

text(0,disjunction(2)-1,0, 'Floor')
text(0,disjunction(2),1, 'Wall')

[X,Y,Z] = cylinder(obs_info(1,3));
mesh(X+obs_info(1,1), Y+obs_info(1,2), Z*2, 'FaceColor', [1 0 0], 'EdgeColor', 'k')

[X,Y,Z] = cylinder(obs_info(2,3));
mesh(X+obs_info(2,1), disjunction(2)-Z*2, Y+obs_info(2,2)-disjunction(2), 'FaceColor', [1 0 0], 'EdgeColor', 'k')

axis equal
grid on
xlim(floor_xy(1:2))
ylim([floor_xy(3) disjunction(2)])
zlim([0 wall_xy(4)-disjunction(2)])
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
%% velocity and position
figure;
subplot(1,2,1)

plot([0:dt:(finish_idx-1)*dt], vs(1,1:finish_idx), '-o')
hold on 
plot([0:dt:(finish_idx-1)*dt], vs(2,1:finish_idx), '-o')
title('Velocity')
legend('V_{x}','V_{y}')
xlabel('time (s)')
ylabel('velocity (m/s)')

subplot(1,2,2)

plot([0:dt:(finish_idx-1)*dt], ps(1,1:finish_idx), '-o')
hold on 
plot([0:dt:(finish_idx-1)*dt], ps(2,1:finish_idx), '-o')
title('Position')
legend('P_{x}','P_{y}')
xlabel('time (s)')
ylabel('position (m)')
