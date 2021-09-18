function visibility_map = buildVisibilityGraph_2D(start, finish, obsInfo, obsInflate)
% This function is to build the global visibility graph
%
% visbility_map = buildVisibilityGraph_2D(start, end, obsInfo, opBox)
% 
% All the obstacles are assumed to be convex as rectangle
% 
% Version 1.0 : Lu, Hong, 13 Aug 2021
% Email: hlu39@sheffield.ac.uk
% Last Modified: 13 Aug 2021

numV = size(obsInfo,1)*4 + 2;

Vertice = cell([numV,1]);

% Adjacant Matrix with weight
adjacantMatrix = ones([numV numV])*Inf;

for i = 1:numV
    Vertice(i) = {idx2obsCoords(i, start, finish, obsInflate)};
end

for i = 1:numV
    for j = i+1:numV
        V_p = Vertice{i};
        V_q = Vertice{j};
        flag = false;
        % check collision between these two
        for o = 1:size(obsInfo,1)
            if isCollideRectangle(V_p, V_q, obsInfo(o,:))
                flag = true;
                break;
            end
        end
        if flag == false 
           adjacantMatrix(i,j) = sqrt(sum((V_p - V_q).^2));
           adjacantMatrix(j,i) = adjacantMatrix(i,j); 
        end
    end
end
visibility_map.Vertice = Vertice;
visibility_map.AdjacantMatrix = adjacantMatrix;
end

function coords = idx2obsCoords(idx, start, finish, obsInfo)
%     each rectangle is indexed as counterclockwise starting from the
%     buttom left vertex.
    if idx == 1
        coords = start;
        return
    elseif idx == 4*size(obsInfo,1) + 2
        coords = finish;
        return
    end

    indexInObs = rem(idx-2,4) + 1;
    indexOfObs = floor((idx-2) / 4) + 1;
    obsinfo = obsInfo(indexOfObs, :);
    switch indexInObs
        case 1
            coords = obsinfo(1:2);
        case 2
            coords = [obsinfo(3), obsinfo(2)];
        case 3
            coords = obsinfo(3:4);
        case 4
            coords = [obsinfo(1), obsinfo(4)];
    end
end