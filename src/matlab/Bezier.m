function Bezier(P, n)
%BEZIER Summary of this function goes here

% Equation of Bezier Curve, utilizes Horner's rule for efficient computation.
% Q(t)=(-P0 + 3*(P1-P2) + P3)*t^3 + 3*(P0-2*P1+P2)*t^2 + 3*(P1-P0)*t + Px0

% control point number = 3

control = zeros(size(P));

control(1,1) = P(1,1);
control(1,2) = P(1,2);
control(2,1) = 3*(2 * P(2,1) - 4/3 * P(1,1));
control(2,2) = 3*(2 * P(2,2) - 4/3 * P(1,2));
control(3,1) = 3*(P(1,1) - 2*P(2,1) + 1/3 * P(3,1));
control(3,2) = 3*(P(1,2) - 2*P(2,2) + 1/3 * P(3,2));

dt = 1/n;
Qx(1) = P(1,1);
Qy(1) = P(1,2);
for i = 1:n
    t = i*dt;
    Qx(i+1) = control(3,1)*t^2 + control(2,1)*t + control(1,1);
    Qy(i+1) = control(3,2)*t^2 + control(2,2)*t + control(1,2);
end

hold on
plot(Qx, Qy, 'g-')

end

