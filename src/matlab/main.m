%% main program for MILP path planning
clear all;
close all;

% add auxilliary folders
addpath ampl_utils\
addpath data\
addpath mod\
addpath output\

%%
% parameters
pos_init    = [0 0];
vel_init    = [0 1];

wind_disturbance = [0 0];

pos_final   = [8 9 8 9];
vel_final   = [0 0];

% waypoints = [6, 7, 6, 7; ...
%              3, 4, 3, 4; ...
%              0, 1, 6, 7;
%              8, 9, 8, 9];

% waypoints = [5, 6, 5, 6; ... 
%              0, 1, 5, 6];

opbox       = [-2 10 -2 10]; % [xmin xmax ymin ymax]

num_vel_cst = 8; % circle approximation number for velocity
num_acc_cst = 8; % circle approximation number for acceleration
num_obs_cst = 8;  % circle approximation number for obstacle

acc_max     = 1; % maximum acceleration
vel_max     = 2; % maximum velocity
vel_min     = 1; % minimum velocity

% obstacle parameters
% obs_info = [2.5, 7.5, 0.5;
%             5.5, 2.5, 0.5;
%             3.5, 4, 0.5;
%             5.5, 5.5, 0.5;
%             1.5, 1.5, 0.5;
%             3, 2, 0.5];

obs_info = [4.0, 5.0, 2.0;
            2.0, 4.0, 1.0];

% obs_info = [2,2,1];

% obs_info = [3, 3, 1;
%             7, 5, 1];

% time parameters
num_ts      = 20;
dt          = 2;

%% MILP for path planning

% data filename
data_name = 'integer_planner_2D';

% planner callback
output_file = integer_planner_2D(pos_init, vel_init, pos_final, vel_final, wind_disturbance, obs_info, opbox, num_vel_cst,...
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

%% MILP for multi-waypoints planning
% 
% % data filename
% data_name   = 'integer_planner_2D_waypoints';
% pos_final = [];
% output_file = integer_planner_2D_waypoints(pos_init, vel_init, waypoints, obs_info, opbox, num_vel_cst,...
%     num_acc_cst, num_obs_cst, acc_max, vel_max, vel_min, num_ts, dt, data_name);
% 
% [flag, ps, vs, as, wps] = analysisSolution(output_file, 'pos', 'vel', 'acc', 'binary_wp');
% 
% if flag
%     disp('GLPK Extraction finished.')
% else
%     disp('GLPK Extraction failed.')
% end
% 
% % should extract where the fs is 1 
% % the time step last till t = [1...fs==1]
% finish_idx = 0;
% for i = 1:size(waypoints, 1)
%     finish_idx = max(finish_idx, find(wps(:,i) == 1));
% end

%% MILP for multi-agent path planning
% 
% pos_init    = [0 0; 6 0; 3 6]';
% vel_init    = [1 0; 0 1; 1 0]';
% 
% pos_final   = [5.5 6 5.5 6; 0.5 1 5.5 6; 2.5 3 0.5 1]';
% 
% opbox       = [-5 10 -5 10]; % [xmin xmax ymin ymax]
% 
% num_vel_cst = 8; % circle approximation number
% num_acc_cst = 8; % circle approximation number
% num_obs_cst = 8;
% num_agt_cst = 4;
% 
% acc_max     = 1;
% vel_max     = 2;
% vel_min     = 1;
% 
% % time parameters
% num_ts      = 30;
% dt          = 0.5;
% 
% % data filename
% data_name   = 'integer_planner_2D_agents';
% 
% output_file = integer_planner_2D_agents(pos_init, vel_init, pos_final, opbox, num_vel_cst,...
%     num_acc_cst, num_agt_cst, acc_max, vel_max, vel_min, num_ts, dt, data_name);
% 
% [flag, ps, vs, as, fs] = analysisSolution(output_file, 'pos', 'vel', 'acc', 'finish');
% 
% if flag
%     disp('GLPK Extraction finished.')
% else
%     disp('GLPK Extraction failed.')
% end
% 
% % extract the finish timing for each agent
% finished_idx = zeros(size(fs,2), 1);
% for i = 1:size(fs,2)
%     finished_idx(i) = find(fs(:,i) == 1);
% end
% 
% %% plot for multi-agent 
% figure;
% 
% color = {validatecolor('#52aa5e');'b'; validatecolor('#A04AD9'); validatecolor('#52aa5e')};
% hold on
% 
% legend_str = {};
% legend_arr = [];
% 
% for i = 1:size(fs,2)
%     px = reshape(ps(1,i,1:finished_idx(i)), [1 finished_idx(i)]);
%     py = reshape(ps(2,i,1:finished_idx(i)), [1 finished_idx(i)]);
%     
%     % draw trajectory
%     p = plot(px, py, '-x', 'Color', color{i});
%     
%     % draw the initial speed
%     endptr = pos_init(:,i) + vel_init(:,i).*dt;
%     initial_speed = [pos_init(:,i) endptr];
%     plot(initial_speed(1,:), initial_speed(2,:), 'k-', 'LineWidth', 2)
%     
%     patch([pos_final(1,i) pos_final(1,i) pos_final(2,i) pos_final(2,i)], [pos_final(3,i) pos_final(4,i) pos_final(4,i) pos_final(3,i)], color{i}, 'FaceAlpha', .4)
%     text(pos_final(2,i), (pos_final(3,i)+pos_final(4,i))/2,  ['Final' num2str(i)])
% 	
%     pth = obstacleApproximation([px(7), py(7), 0.5], num_agt_cst, 1);
%     patch(pth{1}(1,:), pth{1}(2,:), color{i}, 'FaceAlpha', .4)
%     
%     legend_arr(end+1) = p;
%     legend_str(end+1) = {['vehicle' num2str(i)]};
% end
% 
% axis equal
% grid off
% title(['dt = ' num2str(dt) ' s'])
% xlim([-2 8])
% ylim([-2 8])
% xlabel('x (m)')
% ylabel('y (m)')
% 
% legend(legend_arr, legend_str)

%%
figure;

% plot trajectory
plot(ps(1,1:finish_idx), ps(2,1:finish_idx), 'b-x')

hold on

% draw the initial speed
endptr = pos_init + vel_init.*dt;
initial_speed = [pos_init' endptr'];
plot(initial_speed(1,:), initial_speed(2,:), 'k-', 'LineWidth', 2)

if ~isempty(pos_final)
patch([pos_final(1) pos_final(2) pos_final(2) pos_final(1)],[pos_final(3) pos_final(3) pos_final(4) pos_final(4)], 'g')
end

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
if exist('obs_info', 'var') && ~isempty(obs_info)
obs_approx = obstacleApproximation(obs_info, num_obs_cst, ones(size(obs_info,1), 1));

for i = 1:size(obs_info, 1)
    obs_centroid = obs_info(i,1:2);
    obs_radius = obs_info(i,3);
    ox = obs_centroid(1) + obs_radius*cos(linspace(0, 2*pi, 100));
    oy = obs_centroid(2) + obs_radius*sin(linspace(0, 2*pi, 100));
    patch(ox, oy, 'r', 'FaceAlpha', .4)
    approxVertice = obs_approx{i};
    patch(approxVertice(1,:), approxVertice(2,:), 'r', 'FaceAlpha', .4)
end
end

axis equal
grid off
title(['dt = ' num2str(dt) ' s'])
xlim(opbox(1:2))
ylim(opbox(3:4))
xlabel('x (m)')
ylabel('y (m)')

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
