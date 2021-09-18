function localMap = buildLocalMap_2D(p, r, obs_Info)
% This function is to build the local Map
%
% function localMap = buildLocalMap_2D(p, r, obs_Info)
% 
% Version 1.0 : Lu, Hong, 13 Aug 2021
% Email: hlu39@sheffield.ac.uk
% Last Modified: 13 Aug 2021

dl = p-r;
ur = p+r;

localObs = {};

for i = 1:size(obs_Info)
    leftX   = max(dl(1), obs_Info(i,1));
    rightX  = min(ur(1), obs_Info(i,3));
    bottomY = min(ur(2), obs_Info(i,4));
    topY    = max(dl(2), obs_Info(i,2));
    if leftX < rightX && bottomY > topY
       localObs(end+1) = {[leftX,topY,rightX,bottomY]};
    end
end

localMap.obs = localObs;
localMap.boundary = [dl(1) dl(2) ur(1) ur(2)];

end