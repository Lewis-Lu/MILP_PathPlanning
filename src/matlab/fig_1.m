%% figure 1 for dissertation

%% waypoint assginment without/with dynamics constraint

xbound = [-10, 10];
ybound = [-10, 10];

obs_x = {[-3, -3, 3, 3]};
obs_y = {[-1, 2, 2, -1]};

waypoints = {[-5,5],[5,5],[5,-5],[-5,-5]};

start = [-9,0];
terminal = [9,0];

figure;

plot(start(1), start(2), 'Marker', '^','Color', 'r')

hold on

plot(terminal(1), terminal(2), 'Marker', '^','Color', 'k')

for i = 1:length(waypoints)
   plot(waypoints{i}(1), waypoints{i}(2), 'Marker', 'v','Color', 'b')
end

assert(length(obs_x) == length(obs_y), 'dimension of obs_x and obs_y should consistent');

for i = 1:length(obs_x)
    patch(obs_x{i}, obs_y{i}, 'red')
end

% plot unconstraint path
% path = [start; waypoints{1}; waypoints{4}; waypoints{3}; waypoints{2}; terminal];
% plot(path(:,1), path(:,2), '-', 'Color', 'g')

% plot synthesized Bazier curve
Bezier([start; [-7,5]; waypoints{1}], 10);
Bezier([waypoints{1}; [-4,0]; waypoints{4}], 10);
Bezier([waypoints{4}; [0,-5]; waypoints{3}], 10);
Bezier([waypoints{3}; [3,0]; waypoints{2}], 10);
Bezier([waypoints{2}; [5,5]; terminal], 10);

axis equal;
xlabel('x(m)')
ylabel('y(m)')
xlim(xbound);
ylim(ybound);
legend('start','terminal', 'waypoint')