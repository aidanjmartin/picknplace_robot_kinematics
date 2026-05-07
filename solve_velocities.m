function [t3_dot, t4_dot] = solve_velocities(t2, t3, t4, t5, t2_dot, t5_dot, r2, r3, r4, r5)
% SOLVE_VELOCITIES  Angular velocity solver for the 6-bar loop.
%   Differentiates the position loop equations with respect to time and
%   solves the resulting linear system A*x = B for the passive angular
%   velocities t3_dot and t4_dot, given the driven velocities t2_dot and
%   t5_dot.

    A = [-r3*sin(t3), -r4*sin(t4);
          r3*cos(t3),  r4*cos(t4)];

    B = [ r2*sin(t2)*t2_dot - r5*sin(t5)*t5_dot;
         -r2*cos(t2)*t2_dot + r5*cos(t5)*t5_dot];

    x = A \ B;
    t3_dot = x(1);
    t4_dot = x(2);
end
