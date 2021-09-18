function [x,y] = lineIntersection(varargin)
    if nargin < 4
        error('Input four parameters for two lines, repectively. (k1,b1,k2,b2)')
    end
    k1 = varargin{1};
    b1 = varargin{2};
    k2 = varargin{3};
    b2 = varargin{4};
    if k1 == k2 && b1 ~= b2
        error('Two lines are paralleled. No intersection.')
    end
    if k1 == k2 && b1 == b2
        error('Two lines are the same. No intersection.')
    end
    
    x = (b2-b1)/(k1-k2);
    y = k1*x + b1;
end