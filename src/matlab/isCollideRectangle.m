function flag = isCollideRectangle(p, q, obsInfo)
% This function is Collision detection for iterative time selection
%
% function flag = isCollideRectangle(p, q, obsInfo)
% 
% Version 1.0 : Lu, Hong, 11 Aug 2021
% Email: hlu39@sheffield.ac.uk
% Last Modified: 11 Aug 2021

% slope for point p and q
k = (p(2)-q(2))/(p(1)-q(1));
b = q(2) - k*q(1);

% formulate the equality
xmin = obsInfo(1);
ymin = obsInfo(2);
xmax = obsInfo(3);
ymax = obsInfo(4);

ptContact = {};
ptContact(1) = {[xmin, calY(k,xmin,b)]};
ptContact(2) = {[xmax, calY(k,xmax,b)]};
ptContact(3) = {[ymin, calY(k,ymin,b)]};
ptContact(4) = {[ymax, calY(k,ymax,b)]};


for i = 1:length(ptContact)
    xmin_pq = min(p(1), q(1));
    xmax_pq = max(p(1), q(1));
    ymin_pq = min(p(2), q(2));
    ymax_pq = max(p(2), q(2));
    
    pt = ptContact{i};
    
    if xmin_pq <= pt(1) && xmax_pq >= pt(1) &&...
            ymin_pq <= pt(2) && ymax_pq >= pt(2) &&...
                xmin <= pt(1) && xmax >= pt(1) &&...
                    ymin <= pt(2) && ymax >= pt(2)
        flag = true;
        return
    else 
        flag = false;
    end
end

end

% Helper function
function y = calY(k,x,b)
    y = k*x + b;
end