function [X, Y] = get_coordinates(t2, t3, t5, r1, r2, r3, r5, r6)
% GET_COORDINATES  Forward kinematics for plotting the linkage.
%   Computes (X, Y) for every joint along the closed kinematic chain so
%   the linkage can be drawn as a single connected polyline. The slider
%   extension r6 places the end-effector along the coupler axis t3.
%
%   Returned vectors trace: O1 -> J2 -> J5 -> EE -> J5 -> J4 -> O2 -> O1.

    O1x = 0;                  O1y = 0;
    J2x = r2*cos(t2);         J2y = r2*sin(t2);
    J5x = J2x + r3*cos(t3);   J5y = J2y + r3*sin(t3);
    EEx = J5x + r6*cos(t3);   EEy = J5y + r6*sin(t3);

    O2x = r1;                 O2y = 0;
    J4x = O2x + r5*cos(t5);   J4y = O2y + r5*sin(t5);

    X = [O1x, J2x, J5x, EEx, J5x, J4x, O2x, O1x];
    Y = [O1y, J2y, J5y, EEy, J5y, J4y, O2y, O1y];
end
