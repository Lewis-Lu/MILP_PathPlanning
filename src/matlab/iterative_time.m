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

pos_final   = [8 8 9 9];
vel_final   = [0 0];

opbox       = [-2 10 -2 10]; % [xmin xmax ymin ymax]

num_vel_cst = 6; % circle approximation number for velocity
num_acc_cst = 6; % circle approximation number for acceleration
num_obs_cst = 8;  % circle approximation number for obstacle

acc_max     = 1; % maximum acceleration
vel_max     = 2; % maximum velocity
vel_min     = 1; % minimum velocity

% % obstacle parameters
% obs_info = [5.0, 5.0, 1.0;
%             3.0, 3.0, 1.0;
%             3.0, 7.0, 1.0;
%             6.0, 7.0, 1.0];

obs_info = [4.0, 5.0, 2.0;
            2.0, 4.0, 1.0];

obs_inflation = ones(size(obs_info, 1), 1);

% time parameters
num_ts      = 50;
dt          = 0.5;

numIter     = 0;

% Planner History

Planner = {};

%% MILP for path planning

% data filename
data_name   = 'integer_planner_2D_time_selection_first';

% planner callback
output_file = integer_planner_2D_time_selection_first(pos_init, vel_init, pos_final, vel_final, obs_info, opbox, num_vel_cst,...
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


disp(['Total Time Cost :' num2str(finish_idx*dt) ' (s)'])

time_step_finish = floor(finish_idx*dt) + 1;

% Record.p = ps(:,1:finish_idx);
% Record.numIter = numIter;
% Record.timeStamp = [0 time_step_finish];
% Planner(end+1) = {Record};

%% iteratively select the time step
% data filename
data_name   = 'integer_planner_2D_time_selection_iterative';

timeStamp = [];
timeStamp(end+1) = time_step_finish;
timeStamp(end+1) = 0;
num_ts = numel(timeStamp);

output_file = integer_planner_2D_time_selection_iterative(pos_init, vel_init, pos_final, vel_final, obs_info, opbox, num_vel_cst,...
        num_acc_cst, num_obs_cst, acc_max, vel_max, vel_min, num_ts, timeStamp, data_name);

[flag, ps, vs, as, fs] = analysisSolution(output_file, 'pos', 'vel', 'acc', 'finish');

numIter = numIter + 1;
disp(['The Iteration is = ' num2str(numIter)])
disp(['The time stamps array is = ' num2str(timeStamp) ' s.'])

Record.p = ps;
Record.numIter = numIter;
Record.timeStamp = timeStamp;
Record.inflation = obs_inflation;
Planner(end+1) = {Record};

[isCollide, timeStamp] = timeStampGen(ps, obs_info, timeStamp);

while isCollide
    numIter = numIter + 1;
    disp(['The Iteration is = ' num2str(numIter)])
    disp(['The time stamps array is = ' num2str(timeStamp) ' s.'])
    num_ts = numel(timeStamp);
    output_file = integer_planner_2D_time_selection_iterative(pos_init, vel_init, pos_final, vel_final, obs_info, opbox, num_vel_cst,...
        num_acc_cst, num_obs_cst, acc_max, vel_max, vel_min, num_ts, timeStamp, data_name);
    
    [flag, ps, vs, as, fs] = analysisSolution(output_file, 'pos', 'vel', 'acc', 'finish');
    if flag
        disp('GLPK Extraction finished.')
    else
        disp('GLPK Extraction failed.')
    end
    
    Record.p = ps;
    Record.numIter = numIter;
    Record.timeStamp = timeStamp;
    Record.inflation = obs_inflation;
    Planner(end+1) = {Record};
    
    [isCollide, timeStamp] = timeStampGen(ps, obs_info, timeStamp);
end

disp('Time selection Algorithm finished.')


%%
plotIterations(Planner, obs_info, num_obs_cst);

%%
% figure;
% 
% plot(ps(1,:), ps(2,:), 'b-x')
% 
% hold on
% 
% % obstacles
% if exist('obs_info', 'var')
% obs_approx = obstacleApproximation(obs_info, num_obs_cst, 1.0);
% 
% for i = 1:size(obs_info, 1)
%     obs_centroid = obs_info(i,1:2);
%     obs_radius = obs_info(i,3);
%     ox = obs_centroid(1) + obs_radius*cos(linspace(0, 2*pi, 100));
%     oy = obs_centroid(2) + obs_radius*sin(linspace(0, 2*pi, 100));
%     patch(ox, oy, 'r', 'FaceAlpha', .4)
%     approxVertice = obs_approx{i};
%     patch(approxVertice(1,:), approxVertice(2,:), 'r', 'FaceAlpha', .4)
% end
% end
% 
% axis equal
% grid on
% title(['dt = ' num2str(dt) ' s'])
% xlim(opbox(1:2))
% ylim(opbox(3:4))
% xlabel('x (m)')
% ylabel('y (m)')
% 
% %%
% figure;
% 
% % plot trajectory
% plot(ps(1,1:finish_idx), ps(2,1:finish_idx), 'b-x')
% 
% hold on
% 
% % draw the initial speed
% endptr = pos_init + vel_init.*dt;
% initial_speed = [pos_init' endptr'];
% plot(initial_speed(1,:), initial_speed(2,:), 'k-', 'LineWidth', 2)
% 
% patch([pos_final(1) pos_final(3) pos_final(3) pos_final(1)],...
%     [pos_final(2) pos_final(2) pos_final(4) pos_final(4)], 'g')
% 
% % wind disturbance
% if exist('wind_disturbance', 'var')
% interval = 1;
% for i = opbox(3):interval:opbox(4)
%     arrow_base = [opbox(1); i];
%     quiver(arrow_base(1)+1, arrow_base(2)+1, wind_disturbance(1)/2, wind_disturbance(2)/2,...
%         'Color', 'b',...
%         'AutoScale', 'on',...
%         'MaxHeadSize', 2)
% end
% end
% 
% % waypoints 
% if exist('waypoints', 'var')
%     for i = 1:size(waypoints, 1)
%         patch([waypoints(i,1) waypoints(i,1) waypoints(i,2) waypoints(i,2)], [waypoints(i,3) waypoints(i,4) waypoints(i,4) waypoints(i,3)], 'y', 'FaceAlpha', .4)
%         text(waypoints(i,1), (waypoints(i,3)+waypoints(i,4))/2,  ['WP' num2str(i)])
%     end
% end
% 
% % obstacles
% if exist('obs_info', 'var')
% obs_approx = obstacleApproximation(obs_info, num_obs_cst, 1.0);
% 
% for i = 1:size(obs_info, 1)
%     obs_centroid = obs_info(i,1:2);
%     obs_radius = obs_info(i,3);
%     ox = obs_centroid(1) + obs_radius*cos(linspace(0, 2*pi, 100));
%     oy = obs_centroid(2) + obs_radius*sin(linspace(0, 2*pi, 100));
%     patch(ox, oy, 'r', 'FaceAlpha', .4)
%     approxVertice = obs_approx{i};
%     patch(approxVertice(1,:), approxVertice(2,:), 'r', 'FaceAlpha', .4)
% end
% end
% 
% axis equal
% grid on
% title(['dt = ' num2str(dt) ' s'])
% xlim(opbox(1:2))
% ylim(opbox(3:4))
% xlabel('x (m)')
% ylabel('y (m)')

%% velocity and position
% figure;
% subplot(1,2,1)
% 
% plot([0:dt:(finish_idx-1)*dt], vs(1,1:finish_idx), '-o')
% hold on 
% plot([0:dt:(finish_idx-1)*dt], vs(2,1:finish_idx), '-o')
% title('Velocity')
% legend('V_{x}','V_{y}')
% xlabel('time (s)')
% ylabel('velocity (m/s)')
% 
% subplot(1,2,2)
% 
% plot([0:dt:(finish_idx-1)*dt], ps(1,1:finish_idx), '-o')
% hold on 
% plot([0:dt:(finish_idx-1)*dt], ps(2,1:finish_idx), '-o')
% title('Position')
% legend('P_{x}','P_{y}')
% xlabel('time (s)')
% ylabel('position (m)')
