function [t3, t4] = solve_positions(t2, t5, r1, r2, r3, r4, r5)
% SOLVE_POSITIONS  Forward-kinematics position solver for the 6-bar loop.
%   Given the two driven crank angles t2 and t5, returns the passive
%   coupler angle t3 and rocker angle t4 by closing the loop between the
%   two crank tips with the Law of Cosines. The "elbow up" branch is
%   selected so the coupler sits above the line connecting the cranks.

    % Base pivots: left crank at the origin, right crank at (r1, 0)
    O2x = r1;
    O2y = 0;

    % Crank tip positions (J2 = end of left crank, J4 = end of right crank)
    J2x = r2 * cos(t2);
    J2y = r2 * sin(t2);
    J4x = O2x + r5 * cos(t5);
    J4y = O2y + r5 * sin(t5);

    % Vector and length between the two crank tips
    Dx = J4x - J2x;
    Dy = J4y - J2y;
    D = sqrt(Dx^2 + Dy^2);
    theta_D = atan2(Dy, Dx);

    % Interior angle at J2 in the triangle (J2, J5, J4)
    cos_alpha = (r3^2 + D^2 - r4^2) / (2 * r3 * D);
    alpha = acos(real(cos_alpha));

    % Coupler angle (elbow-up branch) and resulting rocker angle
    t3 = theta_D + alpha;
    J5x = J2x + r3 * cos(t3);
    J5y = J2y + r3 * sin(t3);
    t4 = atan2(J4y - J5y, J4x - J5x);
end
