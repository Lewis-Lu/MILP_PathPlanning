%% main program for MILP path planning
close all;
clear all;
clc;

addpath ampl_utils\
addpath data\
addpath mod\
addpath output\

% cleaning img caches
if isunix == true
    system("rm -rf video/img/*");
elseif ispc == true
    system("echo Y | Del /f video\img\* ") % using echo to respond to the command asking.
end

%%
% parameters
pos_init    = [0 0];
vel_init    = [1 0];


pos_final   = [8 9 8 9];
vel_final   = [0 0];

opbox       = [-2 10 -2 10]; % [xmin xmax ymin ymax]

num_vel_cst = 8; % circle approximation number
num_acc_cst = 8; % circle approximation number
num_obs_cst = 4;

acc_max     = 1;
vel_max     = 2;
vel_min     = 1;

% obstacle parameters
obs_info = [2,7,3,8;
            5,2,6,3;
            3,4,4,5;
            5.5,5.5,6.5,6.5;
            1,1,2,2];

% obs_info = [4,4,6,5;
%             5,1,6,4];
        
% time parameters
num_ts      = 50;
dt          = 0.2;

% bounded wind disturbance
% within time steps of num_ts
wind_ub = 0.25;
wind_lb = -0.25;
% wind_disturbance = wind_lb+(wind_ub-wind_lb).*(rand(num_ts, 2));

num_planning_horizon = 5;

% data filename
data_name   = 'integer_planner_2D_MPC';
numIter = 1;

% detection range
detection_range = 2;

% number of the experiment batch
numBatch = 1;

% ROBUSTNESS
% disturbance magnitude
Fdist = 0.1*acc_max;

% form into system
A = [1 dt; 0 1];
B = [0.5*dt*dt; dt];

% make nilpotent controller
K = -acker(A,B,[0 0]);

% margins
dr1 = abs([1 0]*B)*Fdist;
dr2 = dr1 + abs([1 0]*(A+B*K)*B)*Fdist;
dv1 = abs([0 1]*B)*Fdist*sqrt(2);
dv2 = dv1 + abs([0 1]*(A+B*K)*B)*Fdist*sqrt(2);
df1 = abs(K*B)*Fdist*sqrt(2);
df2 = df1 + abs(K*(A+B*K)*B)*Fdist*sqrt(2);

% form up
Rm = [dr1 dr2];
Vm = [dv1 dv2];
Fm = [df1 df2];

Disturbance.Rm = Rm;
Disturbance.Vm = Vm;
Disturbance.Fm = Fm;

POS = {};
VEL = {};
WIND = {};

%% Wind Disturbance Plotting
% figure;
% 
% plot(dt:dt:dt*num_ts, wind_disturbance(:,1), 'm-o')
% hold on 
% plot(dt:dt:dt*num_ts, wind_disturbance(:,2), 'b-o')
% 
% plot(dt:dt:dt*num_ts, ones(1,num_ts).*wind_ub, 'r--')
% plot(dt:dt:dt*num_ts, ones(1,num_ts).*wind_lb, 'r--')
% 
% legend(['X disturbance'; 'Y disturbance'])
% 
% title('Wind disturbance during the time')
% xlim([0 dt*num_ts])
% ylim([-1.5 1.5])
% xlabel('time(s)')
% ylabel('wind velocity (m/s)')

%% Visibility Map Building
% obstacle inflation
obs_inflat = zeros(size(obs_info));
for i = 1:size(obs_info, 1)
    obs_inflat(i,1) = obs_info(i,1) - 0.2;
    obs_inflat(i,2) = obs_info(i,2) - 0.2;
    obs_inflat(i,3) = obs_info(i,3) + 0.2;
    obs_inflat(i,4) = obs_info(i,4) + 0.2;
end

visMap = buildVisibilityGraph_2D(pos_init, [pos_final(1) pos_final(3)], obs_info, obs_inflat);

% dijsktra
shortestPath = dijkstra_vis(visMap);

%% figure for visibility map
% % only for visualization
% figure;
% hold on
% % 
% for i = 1:size(obs_info, 1)
%         ox = [obs_info(i,1) obs_info(i,3) obs_info(i,3) obs_info(i,1)];
%         oy = [obs_info(i,2) obs_info(i,2) obs_info(i,4) obs_info(i,4)];
%         patch(ox, oy, 'r', 'FaceAlpha', .3)
%         ox = [obs_inflat(i,1) obs_inflat(i,3) obs_inflat(i,3) obs_inflat(i,1)];
%         oy = [obs_inflat(i,2) obs_inflat(i,2) obs_inflat(i,4) obs_inflat(i,4)];
%         patch(ox, oy, 'r', 'FaceAlpha', .5)
% end
% 
% adjMatrix = visMap.AdjacantMatrix;
% vertices = visMap.Vertice;
% nV = size(adjMatrix,1);
% % for i = 1:nV
% %     for j = i+1:nV
% %         if adjMatrix(i,j) ~= Inf
% %             p = vertices{i};
% %             q = vertices{j};
% %             plot([p(1) q(1)], [p(2) q(2)], 'b--')
% %         end
% %     end
% % end
% 
% for i = 1:length(shortestPath)-1
%     p = shortestPath{i};
%     q = shortestPath{i+1};
%     plot([p(1) q(1)], [p(2) q(2)], 'b-o', 'LineWidth', 1)
% end
% 
% text(vertices{1}(1)-1.5, vertices{1}(2), 'Start')
% text(vertices{end}(1)+0.5, vertices{end}(2), 'Finish')
% 
% axis equal
% % title('Visibility Graph')
% title({'Shortest Path Extracted sourced from Finish point'})
% xlim(opbox(1:2))
% ylim(opbox(3:4))
% xlabel('x (m)')
% ylabel('y (m)')
% %%%


%% Loop Logic

for batch = 1:numBatch
    
pos = zeros(0,2);
vel = zeros(0,2);
wind = zeros(0,2);

p = pos_init;
v = vel_init;

pos(end+1,:) = p;
vel(end+1,:) = v;

videoFilename = ['video/MPC_' num2str(dt)];
wObj = VideoWriter(videoFilename);
wObj.FrameRate = 10;
open(wObj);

f1 = figure;

isReach = false;

while ~isReach
	
    hold on
    disp(['Iteration ' num2str(numIter) ' start.'])
    
 %%% ========= GLOBAL ENVIRONMENT PLOTTING ===================
    
    % target box
    patch([pos_final(1) pos_final(1) pos_final(2) pos_final(2)],[pos_final(3) pos_final(4) pos_final(4) pos_final(3)], 'g')
    
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
    
    % global obstacles
    if ~isempty(obs_info) || size(obs_info,1) > 0
        for i = 1:size(obs_info, 1)
                ox = [obs_info(i,1) obs_info(i,3) obs_info(i,3) obs_info(i,1)];
                oy = [obs_info(i,2) obs_info(i,2) obs_info(i,4) obs_info(i,4)];
                patch(ox, oy, 'k', 'FaceAlpha', .2)
                ox = [obs_inflat(i,1) obs_inflat(i,3) obs_inflat(i,3) obs_inflat(i,1)];
                oy = [obs_inflat(i,2) obs_inflat(i,2) obs_inflat(i,4) obs_inflat(i,4)];
                patch(ox, oy, 'k', 'FaceAlpha', .1)
        end
    end
%%% ================= GLOBAL ENVIRONMENT PLOTTING END =================

    % draw the speed
    endptr = p + v.*dt;
    initial_speed = [p' endptr'];
    plot(initial_speed(1,:), initial_speed(2,:), 'k-', 'LineWidth', 2)

    % build the local map for vehicle
    % local map containing local obstacles and local horizon boundary
    localMap = buildLocalMap_2D(p, detection_range, obs_inflat);
    localBdy = localMap.boundary;
    
    patch([localBdy(1) localBdy(3) localBdy(3) localBdy(1)],...
        [localBdy(2) localBdy(2) localBdy(4) localBdy(4)],...
        'r',...
        'FaceAlpha', .3, ...
        'LineStyle', 'none')
    
    localObs = zeros(length(localMap.obs), 4);
    % struct from CELL to ARRAY
    for i = 1:length(localMap.obs)
        localObs(i,:) = localMap.obs{i};
    end
    % PLOT LOCAL SENSED OBSTACLE
    for i = 1:size(localObs, 1)
        ox = [localObs(i,1) localObs(i,3) localObs(i,3) localObs(i,1)];
        oy = [localObs(i,2) localObs(i,2) localObs(i,4) localObs(i,4)];
        patch(ox, oy, 'r', 'FaceAlpha', .5)
    end
    
%%% ================= LOCAL ENVIRONMENT PLOTTING END =================

    % disturbance
    thd = 2*pi*rand(1,1);
    d = Fdist*rand(1,1)*[cos(thd) sin(thd)];

    % The path planning should be executed within the local map horizon
    % Feed with the local obstacle for avoidance
    output_file = integer_planner_2D_MPC(p,...
                                        v,...
                                        pos_final,...
                                        localObs,...
                                        opbox,...
                                        num_vel_cst, ...
                                        num_acc_cst,...
                                        acc_max,...
                                        vel_max,...
                                        vel_min,...
                                        num_ts,...
                                        num_planning_horizon,...
                                        dt,...
                                        Disturbance,...
                                        d,...
                                        data_name);
    
    [flag, ps, vs, as, fs] = analysisSolution(output_file, 'pos', 'vel', 'acc', 'finish');
    
    if flag
        disp('GLPK Extraction finished.')
        % should extract where the fs is 1, the time step last till t = [1...fs==1]
%         finish_idx = find(fs == 1);
        ps = ps(:,1:num_planning_horizon);
        vs = vs(:,1:num_planning_horizon);
        as = as(:,1:num_planning_horizon);

        % plot predicted trajectory
        plot(ps(1,1:num_planning_horizon), ps(2,1:num_planning_horizon), 'b-x')
        
        pos(end+1,:)    = p;
        vel(end+1,:)    = v;
        wind(end+1,:)   = d;
        
        axis equal
        grid off
        titleName = sprintf('Time = %1.1f (s), dt = %1.1f (s)', dt*numIter, dt);
        title(titleName)
        xlim(opbox(1:2))
        ylim(opbox(3:4))
        xlabel('x (m)')
        ylabel('y (m)')
        fr = getframe(f1);
        writeVideo(wObj,fr);
        
        numIter = numIter+1;
        
        % control sigularity here?
        % Solver cannot solve this after exactly 3 iterations.
        
%         p = p + v.*dt + 0.5*dt*dt.*(a+d);
%         v = v + (v+d).*dt;
        
        p = ps(:,2)';
        v = vs(:,2)';
        
        isReach = isGoal(p, pos_final);
        
    else
        disp('GLPK Extraction failed.')
        close(wObj);
        return 
    end
    
    saveas(f1,['video/img/MPC_dt_' num2str(dt) '_idx_' num2str(numIter) '.jpg'])
    
    clf
end

POS(end+1) = {pos};
VEL(end+1) = {vel};
WIND(end+1) = {wind};

close(wObj);
close(f1);
disp(['Batch ' num2str(numBatch) ' finished.'])

end

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
