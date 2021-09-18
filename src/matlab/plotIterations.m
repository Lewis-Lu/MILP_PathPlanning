function plotIterations(Planner, obsInfo, numObsCst)
% 
% Plotting Utility for the Iteration MILP Algorithms 
% 
% Version 1.0 : Lu, Hong, 17 July 2021
% Email: hlu39@sheffield.ac.uk
% Last Modified: 22 Aug 2021

numIter = numel(Planner);
numCol = 3;
numRow = floor(numIter/numCol)+1;

opbox       = [-2 10 -2 10]; % [xmin xmax ymin ymax]
obs_info    = obsInfo;
num_obs_cst = numObsCst;

fig = figure;
figPos = fig.Position;

for n = 1:numIter
    subplot(numRow, numCol, n)
    
    ps = Planner{n}.p;
    
    plot(ps(1,:), ps(2,:), 'b-x')
    
    hold on
    
    % obstacles
    if size(obs_info,1) ~= 0
        obs_approx = obstacleApproximation(obs_info, num_obs_cst, Planner{n}.inflation);
        for i = 1:size(obs_info, 1)
            obs_centroid = obs_info(i,1:2);
            obs_radius = obs_info(i,3);
            ox = obs_centroid(1) + obs_radius*cos(linspace(0, 2*pi, 100));
            oy = obs_centroid(2) + obs_radius*sin(linspace(0, 2*pi, 100));
            patch(ox, oy, 'r', 'FaceAlpha', .4)
            approxVertice = obs_approx{i};
            patch(approxVertice(1,:), approxVertice(2,:), 'r', 'FaceAlpha', .4, 'LineStyle', '--')
        end
    end
    
    axis equal
    grid off
    title(['Iteration = ' num2str(Planner{n}.numIter)])
    xlim(opbox(1:2))
    ylim(opbox(3:4))
    xlabel('x (m)')
    ylabel('y (m)')
    
end

 
% figure;
% ps = Planner{end}.p;
% plot(ps(1,:), ps(2,:), 'b-x')
%     
% hold on
% 
% % obstacles
% if size(obs_info,1) ~= 0
%     obs_approx = obstacleApproximation(obs_info, num_obs_cst, Planner{end}.inflation);
%     for i = 1:size(obs_info, 1)
%         obs_centroid = obs_info(i,1:2);
%         obs_radius = obs_info(i,3);
%         ox = obs_centroid(1) + obs_radius*cos(linspace(0, 2*pi, 100));
%         oy = obs_centroid(2) + obs_radius*sin(linspace(0, 2*pi, 100));
%         patch(ox, oy, 'r', 'FaceAlpha', .4)
%         approxVertice = obs_approx{i};
%         patch(approxVertice(1,:), approxVertice(2,:), 'r', 'FaceAlpha', .4, 'LineStyle', '--')
%     end
% end
% 
% axis equal
% grid off
% title(['Iteration = ' num2str(Planner{end}.numIter)])
% xlim(opbox(1:2))
% ylim(opbox(3:4))
% xlabel('x (m)')
% ylabel('y (m)')

end