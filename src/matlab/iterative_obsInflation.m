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
% obs_info = [2.0, 3.0, 1.0;
%             6.0, 4.0, 1.5;
%             3.5, 5.0, 1.0;
%             5.0, 7.0, 1.0];
       
obs_info = [4.0, 5.0, 2.0;
            2.0, 4.0, 1.0];

obs_inflation = ones(size(obs_info, 1), 1);

obs_inflation_alpha = 0.1;
             
% time parameters
num_ts      = 50;
dt          = 0.5;

% Planner History

Planner = {};

%% MILP for path planning

% data filename
data_name   = 'integer_planner_2D_obstacle_inflation';

timeStamp           = [];
timeStamp(end+1)    = 25;
timeStamp(end+1)    = 0;
timeStamp           = sort(timeStamp);

num_ts              = numel(timeStamp);
numIter             = 0;

% % planner callback
output_file = integer_planner_2D_obstacle_inflation(pos_init, vel_init, pos_final, vel_final, obs_info, opbox, num_vel_cst,...
    num_acc_cst, num_obs_cst, acc_max, vel_max, vel_min, num_ts, timeStamp, obs_inflation, data_name);

% analyse the result output file
[flag, ps, vs, as, fs] = analysisSolution(output_file, 'pos', 'vel', 'acc', 'finish');
if flag
    disp('GLPK Extraction finished.')
else
    disp('GLPK Extraction failed.')
end

[isCollide, ~, obs_index] = timeStampGen(ps, obs_info, timeStamp);

Record.p = ps;
Record.numIter = numIter;
Record.timeStamp = timeStamp;
Record.inflation = obs_inflation;
Record.obsIndex = obs_index;
Planner(end+1) = {Record};

% obstacle inflation
for i = 1:numel(obs_index)
    obs_inflation(obs_index(i)) = obs_inflation(obs_index(i)) + obs_inflation_alpha;
end

while isCollide
    numIter = numIter + 1;
    
    disp(['The Iteration is = ' num2str(numIter)])
    disp(['The time stamps array is = ' num2str(timeStamp) ' s.'])
    
    % obstacle inflation
    for i = 1:numel(obs_index)
        obs_inflation(obs_index(i)) = obs_inflation(obs_index(i)) + obs_inflation_alpha;
    end
    
    num_ts = numel(timeStamp);
    output_file = integer_planner_2D_obstacle_inflation(pos_init, vel_init, pos_final, vel_final, obs_info, opbox, num_vel_cst,...
        num_acc_cst, num_obs_cst, acc_max, vel_max, vel_min, num_ts, timeStamp, obs_inflation, data_name);

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
    Record.obsIndex = obs_index;
    Planner(end+1) = {Record};
    
    [isCollide, timeStamp, obs_index] = timeStampGen(ps, obs_info, timeStamp);
end

disp('Time selection Algorithm finished.')

%%
plotIterations(Planner, obs_info, num_obs_cst);

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
