function [flag, collidePairs] = isCollisionIterative(path, obsInfo)
% This function return flag indicator for iterative-algorithm collsiion
% check
%
% function [flag, collidePairs] = isCollisionIterative(path, obsInfo)
% 
% Version 1.0 : Lu, Hong, 17 July 2021
% Email: hlu39@sheffield.ac.uk
% Last Modified: 17 Aug 2021

numVertices = size(path, 1);
numObstacle = size(obsInfo, 1);
collidePairs = {};
flag = false;

% this function checked the piece-wise collision along the given path
for i = 1:numVertices-1
    for k = 1:numObstacle
        if isCollideCircle(path(i,:), path(i+1,:), obsInfo(k,:))
%             [~,isInCircle] = isCollideCircle(path(i,:), path(i+1,:), obsInfo(k,:));
            collidePairs(end+1) = {[i,i+1]};
            flag = true;
            break;
        end
    end
end

end