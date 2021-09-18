%% main program for MILP path planning
clear all;
close all;

addpath ampl_utils\
addpath data\
addpath mod\
addpath output\

%% Initialization
% parameters
pos_init    = [0 0];
vel_init    = [1 0];

pos_final   = [6 6];
% vel_final   = [0 0];

opbox       = [-2 8 -2 8]; % [xmin xmax ymin ymax]

num_vel_cst = 8; % circle approximation number
num_acc_cst = 8; % circle approximation number
num_obs_cst = 8; % circle approximation number

acc_max     = 1;
vel_max     = 2;
vel_min     = 1;

% obstacle parameters
obs_info = [3.5,3.5,0.8;
            2,2,0.5];

obs_inflate_alpha = 1;
        
% time parameters
num_ts      = 20;
dt          = 2;

% data filename
data_name   = 'integer_planner_2D_time_iterative';
numIter     = 1;

%% firstly coarse planning using MILP
indicator = 1;
output_file = integer_planner_2D_iterative_time(indicator, pos_init, vel_init, pos_final, obs_info, opbox, num_vel_cst,...
    num_acc_cst, num_obs_cst, acc_max, vel_max, vel_min, num_ts, dt, data_name);

[flag, ps, vs, as, fs] = analysisSolution(output_file, 'pos', 'vel', 'acc', 'finish');
if flag
    disp('GLPK Extraction finished. Coarsely Planning 1st time')
else
    disp('GLPK Extraction failed.')
end

% should extract where the fs is 1, the time step last till t = [1...fs==1]
finish_idx = find(fs == 1);
ps = ps(:,1:finish_idx);
vs = vs(:,1:finish_idx);
as = as(:,1:finish_idx);

%% Iterative Time selection algorithm

figure;

hold on
axis equal

if ~isempty(obs_info) || size(obs_info,1) > 0
    obs_approx = obstacleApproximation(obs_info, num_obs_cst, obs_inflate_alpha);
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



% start second iteration
indicator = 0;
formerPath = ps';

[isCollide_flag, collisionSet] = isCollisionIterative(formerPath, obs_info);

while isCollide_flag
    
    plot(formerPath(:,1), formerPath(:,2), 'b-x')
    pause
    numIter = numIter + 1;
    
    % merge the connected collision path
    % [vertex 1, vertex 2, edge status = inCircle/notInCircle]
    collide = {};
    pc_start = collisionSet{1}(1);
    i = 1;
    while i < length(collisionSet)
        p = collisionSet{i}(2);
        q = collisionSet{i+1}(1);
        if p ~= q
            collide(end+1) = {[pc_start, p]};
            pc_start = q;
        end
        i = i+1;
    end
    collide(end+1) = {[pc_start, collisionSet{i}(2)]};
    
    % construct struct for collision paths
    Replanner = struct;
    
    Replanner.num_path  = length(collide);
    Replanner.path      = collide; % cell array
    Replanner.dt        = dt/2;
    dt = dt/2;
    Replanner.pos_prior_start = zeros(length(collide), 2);
    Replanner.vel_prior_start = zeros(length(collide), 2);
    Replanner.pos_prior_terminate = zeros(length(collide), 2);
    Replanner.vel_prior_terminate = zeros(length(collide), 2);
    for i = 1:length(collide)
        Replanner.pos_prior_start(i,:)      = ps(:,collide{i}(1)); % constrained start position
        Replanner.pos_prior_terminate(i,:)  = ps(:,collide{i}(2)); % constrained terminal position
        Replanner.vel_prior_start(i,:)      = vs(:,collide{i}(1)); % constrained start velocity
        Replanner.vel_prior_terminate(i,:)  = vs(:,collide{i}(2)); % constrained terminal velocity
    end
    Replanner.formerPath    = ps;
    Replanner.formerVel     = vs;
    Replanner.formerAcc     = as;
    Replanner.replannedPos  = cell(length(collide), 1);
    Replanner.replannedVel  = cell(length(collide), 1);
    Replanner.replannedAcc  = cell(length(collide), 1);
    
    % solve the augumented MILP
    for i = 1:Replanner.num_path
        output_file = integer_planner_2D_iterative_time(indicator,...
            Replanner.pos_prior_start(i,:), ...
            [0 0],...
            Replanner.pos_prior_terminate(i,:),...
            obs_info,...
            opbox,...
            num_vel_cst,...
            num_acc_cst,...
            num_obs_cst, ...
            acc_max,...
            vel_max,...
            vel_min,...
            num_ts,...
            Replanner.dt,...
            data_name);
        
        % analyse the result file
        [flag, ps, vs, as, fs] = analysisSolution(output_file, 'pos', 'vel', 'acc', 'finish');
        
        if flag
            disp('GLPK Extraction finished.')
        else
            disp(['GLPK Extraction failed. Iteration = ' num2str(numIter) ' #Subpath = ' num2str(i)])
            return
        end
        
         finish_idx = find(fs == 1);
         Replanner.replannedPos{i} = ps(:,1:finish_idx)';
         Replanner.replannedVel{i} = vs(:,1:finish_idx)';
         Replanner.replannedAcc{i} = as(:,1:finish_idx)';
    end
    
    fusedPath = zeros(0,2);
%   mapping the Replanner into formerPath;
    collideIdx = 1;
    collidePathIdx = collide{collideIdx};
    collidePathStartIdx = collidePathIdx(1);
    i = 1;
    while i <= size(formerPath, 1)
        if i == collidePathStartIdx
           % deal with the replanned path 
           replanned_len = size(Replanner.replannedPos{collideIdx},1);
           fusedPath(end+1:end+replanned_len,:) = Replanner.replannedPos{collideIdx};
           i = collidePathIdx(2)+1;
           if collideIdx ~= length(collide)
               collideIdx = collideIdx+1;
               collidePathIdx = collide{collideIdx};
               collidePathStartIdx = collidePathIdx(1);    
           end
        else
            fusedPath(end+1,:) = formerPath(i,:);
            i = i+1;
        end
    end
    formerPath = fusedPath;
    [isCollide_flag, collisionSet] = isCollisionIterative(formerPath, obs_info);
end

axis equal
grid off
% title(['Iteration = ' num2str(numIter)])
xlim(opbox(1:2))
ylim(opbox(3:4))
xlabel('x (m)')
ylabel('y (m)')

%%
figure;

% plot trajectory
plot(formerPath(:,1), formerPath(:,2), 'b-x')

hold on

% draw the initial speed
endptr = pos_init + vel_init.*dt;
initial_speed = [pos_init' endptr'];
plot(initial_speed(1,:), initial_speed(2,:), 'k-', 'LineWidth', 2)

final_margin = 0.5;
patch([pos_final(1) pos_final(1)+final_margin pos_final(1)+final_margin pos_final(1)],...
    [pos_final(2) pos_final(2) pos_final(2)+final_margin pos_final(2)+final_margin], 'g')

% wind disturbance
if exist('wind_disturbance', 'var')
interval = 1;
for i = opbox(3):interval:opbox(4)
    arrow_base = [opbox(1); i];
    quiver(arrow_base(1)+1, arrow_base(2)+1, wind_disturbance(1)/2, wind_disturbance(2)/2,...
        'Color', 'b',...
        'AutoScale', 'on',...
        'MaxHeadSize', 2)
end
end

% waypoints 
if exist('waypoints', 'var')
    for i = 1:size(waypoints, 1)
        patch([waypoints(i,1) waypoints(i,1) waypoints(i,2) waypoints(i,2)], [waypoints(i,3) waypoints(i,4) waypoints(i,4) waypoints(i,3)], 'y', 'FaceAlpha', .4)
        text(waypoints(i,1), (waypoints(i,3)+waypoints(i,4))/2,  ['WP' num2str(i)])
    end
end

% obstacles
if ~isempty(obs_info) || size(obs_info,1) > 0
    obs_approx = obstacleApproximation(obs_info, num_obs_cst, obs_inflate_alpha);
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
grid on
title(['Iteration = ' num2str(numIter)])
xlim(opbox(1:2))
ylim(opbox(3:4))
xlabel('x (m)')
ylabel('y (m)')

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
