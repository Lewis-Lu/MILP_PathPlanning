function occ_map = buildOccupancy_2D(opBox, obsInfo)
% This function is 2D occupancy map build tool
%
% function occ_map = buildOccupancy_2D(opBox, obsInfo)
% 
% Version 1.0 : Lu, Hong, 11 Aug 2021
% Email: hlu39@sheffield.ac.uk
% Last Modified: 11 Aug 2021

nx = abs(opBox(1) - opBox(3));
ny = abs(opBox(2) - opBox(4));

occ_map = logical(zeros(nx, ny));

for i = 1:size(obsInfo, 1)
    % convert the coordinates from map to occ_map
    obs_coords = obsInfo(i,:);
    for j = 1:2
        obs_coords(2*j-1) = obs_coords(2*j-1) - opBox(2*j-1);
        obs_coords(2*j) = obs_coords(2*j) - opBox(2*j-1);
    end
%     convert the logic 0 to 1;
    occ_map(obs_coords(2):obs_coords(4), obs_coords(1):obs_coords(3)) = true;
end

end