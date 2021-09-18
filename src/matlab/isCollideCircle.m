function flag = isCollideCircle(p, q, obsInfo)
% This function is Collision detection for iterative time selection
%
% function flag = isCollideCircle(p, q, obsInfo)
% 
% Version 1.0 : Lu, Hong, 7 Aug 2021
% Email: hlu39@sheffield.ac.uk
% Last Modified: 7 Aug 2021

% slope for point p and q
k = (p(2)-q(2))/(p(1)-q(1));
b = q(2) - k*q(1);

% formulate the equality
ox = obsInfo(1);
oy = obsInfo(2);
or = obsInfo(3);

if (p(1)-ox)^2 + (p(2)-oy)^2 <= or^2 || (q(1)-ox)^2 + (q(2)-oy)^2 <= or^2
   flag = true;
%    isInCircle = 1;
   return
end

A = 1+k^2;
B = -2*ox + 2*b*k - 2*oy*k;
C = ox^2 + (b-oy)^2 - or^2;

delta_squre = B^2 - 4*A*C;

if delta_squre < 0
    disp('isCollideObstacle: Delta Value < 0, No Contact Point')
    flag = false; 
    return 
end

ptContact = {};

if delta_squre == 0
    xContact = -B/(2*A);
    ptContact{1} = {[xContact; calY(k,xContact,b)]};
elseif delta_squre > 0
    xContact_lo = (-B-sqrt(delta_squre))/(2*A);
    xContact_hi = (-B+sqrt(delta_squre))/(2*A);
    ptContact{1} = ([xContact_lo; calY(k,xContact_lo,b)]);
    ptContact{2} = ([xContact_hi; calY(k,xContact_hi,b)]);
end

for i = 1:length(ptContact)
    xmin = min(p(1), q(1));
    xmax = max(p(1), q(1));
    ymin = min(p(2), q(2));
    ymax = max(p(2), q(2));
    pt = ptContact{i};
    if xmin <= pt(1) && xmax >= pt(1) && ymin <= pt(2) && ymax >= pt(2)
        flag = true;
        return
    else 
        flag = false;
    end
end

flag = false;

% isInCircle = 0;

end

function y = calY(k,x,b)
    y = k*x + b;
end