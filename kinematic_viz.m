% kinematic_viz.m
% ENGR 3590 - Project 3: 6-Bar Pick-and-Place Robot
% Renders a single static pose of the linkage for use in the report.
clear; clc; close all;

%% Link lengths (inches)
r1 = 4;  % ground link (distance between the two base pivots)
r2 = 5;  % left crank
r3 = 6;  % left coupler
r4 = 6;  % right rocker
r5 = 5;  % right crank

%% Static pose target
% Place joint J5 (the "elbow") at a position that gives the same
% reach-up-and-left silhouette used in the report figures.
J5_X = -0.5;
J5_Y = 6.5;

%% Kinematics
[t2, t3, t4, t5] = solve_IK(J5_X, J5_Y, r1, r2, r3, r4, r5);

% Prismatic slider extension along the coupler axis (inches)
r6 = 1.5;

[X_path, Y_path] = get_coordinates(t2, t3, t5, r1, r2, r3, r5, r6);

%% Figure setup
figure('Name', 'Linkage Static Pose', 'Position', [100, 100, 1100, 600]);
hold on; axis equal;
axis([-6 10 -2 12]);
title('6-Bar Linkage: Static Presentation Pose');
xlabel('X Position (in)'); ylabel('Y Position (in)');
set(gca, 'Color', [0.91, 0.93, 0.95]);

%% Linkage bars and joints
plot(X_path([1, 2, 3, 6, 7]), Y_path([1, 2, 3, 6, 7]), ...
    'Color', [0.20, 0.29, 0.37], 'LineStyle', '-', 'LineWidth', 12, ...
    'Marker', 'o', 'MarkerSize', 14, ...
    'MarkerFaceColor', [0.17, 0.24, 0.31], 'MarkerEdgeColor', 'k');

%% Prismatic slider rod (yellow), extending along the coupler axis t3
plot([X_path(3), X_path(4)], [Y_path(3), Y_path(4)], ...
    'Color', [0.95, 0.77, 0.06], 'LineWidth', 8);

%% Rotating fork gripper (yellow), aligned with the slider axis
fork_W = 1.5; fork_L = 1.5;

dx_perp = -sin(t3) * (fork_W/2); dy_perp =  cos(t3) * (fork_W/2);
dx_par  =  cos(t3) * fork_L;     dy_par  =  sin(t3) * fork_L;

F1x = X_path(4) + dx_perp; F1y = Y_path(4) + dy_perp;
F2x = X_path(4) - dx_perp; F2y = Y_path(4) - dy_perp;
F3x = F1x + dx_par;        F3y = F1y + dy_par;
F4x = F2x + dx_par;        F4y = F2y + dy_par;

plot([F3x, F1x, F2x, F4x], [F3y, F1y, F2y, F4y], ...
    'Color', [0.95, 0.77, 0.06], 'LineWidth', 8);
