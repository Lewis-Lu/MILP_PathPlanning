function [flag, newTimeStamp, obs_index] = timeStampGen(path, obsInfo, timeStamp)
% 
% function [flag, newTimeStamp] = timeStampGen(path, obsInfo)
% 
% Version 1.0 : Lu, Hong, 17 July 2021
% Email: hlu39@sheffield.ac.uk
% Last Modified: 22 Aug 2021

numVertex = size(path, 2);
numObs = size(obsInfo, 1);
newTimeStamp = timeStamp;
obs_index = [];

flag = false;

for i = 2:numVertex
    for j = 1:numObs
        isCollide = isCollideCircle(path(:,i-1), path(:,i), obsInfo(j,:));
        if isCollide
            newTime = (timeStamp(i) + timeStamp(i-1)) / 2;
            newTimeStamp(end+1) = newTime;
            obs_index(end+1) = j;
            flag = true;
        end
    end
end

obs_index = unique(obs_index);
newTimeStamp = unique(sort(newTimeStamp));

end