function [t2, t3, t4, t5] = solve_IK(X, Y, r1, r2, r3, r4, r5)
% SOLVE_IK  Inverse kinematics for the 6-bar pick-and-place linkage.
%   Given a target end-effector position (X, Y) at joint J5 and the link
%   lengths, returns the four joint angles required to reach it. The
%   linkage is decoupled into a left arm (r2, r3) anchored at the origin
%   and a right arm (r5, r4) anchored at (r1, 0). Each arm is solved
%   independently using the Law of Cosines.

    % Left arm: distance from origin to target, then crank angle t2
    D1 = sqrt(X^2 + Y^2);
    alpha1 = acos(real((r2^2 + D1^2 - r3^2) / (2 * r2 * D1)));
    t2 = atan2(Y, X) + alpha1;

    % Right arm: distance from (r1, 0) to target, then crank angle t5
    X2 = X - r1;
    D2 = sqrt(X2^2 + Y^2);
    alpha2 = acos(real((r5^2 + D2^2 - r4^2) / (2 * r5 * D2)));
    t5 = atan2(Y, X2) - alpha2;

    % Crank tip positions used to recover the passive coupler/rocker angles
    J2x = r2 * cos(t2);     J2y = r2 * sin(t2);
    J4x = r1 + r5*cos(t5);  J4y = r5 * sin(t5);

    t3 = atan2(Y - J2y, X - J2x);  % left coupler angle
    t4 = atan2(Y - J4y, X - J4x);  % right rocker angle
end
