function path = dijkstra_vis(visMap)
% This function is to extract the shortest path in the adjacant matrix
%
% function path = dijkstra(visMap)
% 
% Version 1.0 : Lu, Hong, 13 Aug 2021
% Email: hlu39@sheffield.ac.uk
% Last Modified: 13 Aug 2021

if ~isfield(visMap, 'AdjacantMatrix')
    error('Visibility Map do not have member AdjacantMatrix')
end

adjMat = visMap.AdjacantMatrix;
% number of vertices
numV = size(adjMat,1);
% visited array for recording
visited = logical(zeros(numV ,1));
% preceeding array for preorder record
preceedingVertex = zeros(numV, 1);
% cost array for recording the cost from the target point
cost = ones(numV, 1)*Inf;
% search start from the goal
cost(end) = 1e8;

while sum(visited) ~= numV
    unvisited_idx = (find(visited == false));
    tmp = Inf;
    nextV_idx = Inf;
    for i = 1:size(unvisited_idx,1)
        if cost(unvisited_idx(i)) < tmp
            tmp = cost(unvisited_idx(i));
            nextV_idx = unvisited_idx(i);
        end
    end
%     disp(['next Vertice to search is ' num2str(nextV_idx)])
%     update the cost and preceeding vertex
    for i = 1:numV
        if adjMat(nextV_idx, i) ~= Inf
            if cost(i) > cost(nextV_idx) + adjMat(nextV_idx,i)
                cost(i) = cost(nextV_idx) + adjMat(nextV_idx,i);
                preceedingVertex(i) = nextV_idx;
            end
        end
    end
    visited(nextV_idx) = true;
end

path = {};
p = 1;
path(end+1) = visMap.Vertice(p);
while p ~= numV
    p = preceedingVertex(p);
    path(end+1) = visMap.Vertice(p);
end

end