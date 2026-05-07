% main.m
% ENGR 3590 - Project 3: 6-Bar Pick-and-Place Robot
%
% Drives the full pick-and-place animation. For each box the script:
%   1. Scans the reachable workspace (geometrically validated against the
%      arm-length triangle inequalities) for an elbow position J5 that
%      lets the prismatic slider reach the target along the coupler axis.
%   2. Builds a piecewise-linear trajectory through the six phases of the
%      cycle (plunge down, retract, traverse, plunge down, retract, return).
%   3. Solves inverse kinematics each frame and renders the linkage,
%      slider, gripper fork, and the boxes being moved.
clear; clc; close all;

%% Link lengths (inches)
r1 = 4;  % ground link
r2 = 5;  % left crank
r3 = 6;  % left coupler
r4 = 6;  % right rocker
r5 = 5;  % right crank

%% Animation / scene parameters
frames_per_phase = 40;
num_boxes = 3;

% Pick locations (top conveyor; constant Y)
T_pick_X = [-4.0, -2.5, -1.0];
T_pick_Y = 9.0;

% Drop locations (lower conveyor; flat in this build)
T_drop_X = [14.0, 15.5, 17.0];
T_drop_Y = [6.0, 6.0, 6.0];

%% Workspace scanner
% For every target the scanner sweeps candidate elbow positions (J5) and
% picks the one whose required slider extension r6 = dY / sin(t3) lands
% the gripper tip closest to the target X. Triangle-inequality bounds on
% each arm reject any J5 that is geometrically unreachable, which is what
% prevents acos() from returning complex numbers later.
pick_J5_Xs = zeros(1, num_boxes); pick_J5_Ys = zeros(1, num_boxes);
drop_J5_Xs = zeros(1, num_boxes); drop_J5_Ys = zeros(1, num_boxes);
r6_pick_req = zeros(1, num_boxes); r6_drop_req = zeros(1, num_boxes);

max_L = r2 + r3; min_L = abs(r2 - r3);
max_R = r4 + r5; min_R = abs(r4 - r5);

for b = 1:num_boxes
    % Pick side
    best_err = inf; best_X = 0; best_Y = 0; best_r6 = 0;
    for test_X = -4:0.05:4
        for test_Y = 1:0.05:8
            dist_L = sqrt(test_X^2 + test_Y^2);
            dist_R = sqrt((test_X - r1)^2 + test_Y^2);

            if (dist_L <= max_L) && (dist_R <= max_R) && (dist_L >= min_L) && (dist_R >= min_R)
                [~, t3_p, ~, ~] = solve_IK(test_X, test_Y, r1, r2, r3, r4, r5);
                if isreal(t3_p)
                    r6_p = (T_pick_Y - test_Y) / sin(t3_p);
                    if r6_p > 1 && r6_p < 20  % slider must extend forward, not retract
                        tip_X = test_X + r6_p * cos(t3_p);
                        if abs(tip_X - T_pick_X(b)) < best_err
                            best_err = abs(tip_X - T_pick_X(b));
                            best_X = test_X; best_Y = test_Y; best_r6 = r6_p;
                        end
                    end
                end
            end
        end
    end
    pick_J5_Xs(b) = best_X; pick_J5_Ys(b) = best_Y; r6_pick_req(b) = best_r6;

    % Drop side
    best_err = inf; best_X = 0; best_Y = 0; best_r6 = 0;
    for test_X = 0:0.05:8
        for test_Y = 1:0.05:5.5
            dist_L = sqrt(test_X^2 + test_Y^2);
            dist_R = sqrt((test_X - r1)^2 + test_Y^2);

            if (dist_L <= max_L) && (dist_R <= max_R) && (dist_L >= min_L) && (dist_R >= min_R)
                [~, t3_d, ~, ~] = solve_IK(test_X, test_Y, r1, r2, r3, r4, r5);
                if isreal(t3_d)
                    r6_d = (T_drop_Y(b) - test_Y) / sin(t3_d);
                    if r6_d > 1 && r6_d < 20
                        tip_X = test_X + r6_d * cos(t3_d);
                        if abs(tip_X - T_drop_X(b)) < best_err
                            best_err = abs(tip_X - T_drop_X(b));
                            best_X = test_X; best_Y = test_Y; best_r6 = r6_d;
                        end
                    end
                end
            end
        end
    end
    drop_J5_Xs(b) = best_X; drop_J5_Ys(b) = best_Y; r6_drop_req(b) = best_r6;
end

%% Trajectory assembly
% Six phases per box: plunge to pick, retract, traverse to drop, plunge
% to drop, retract, traverse to next pick. Each phase is `frames_per_phase`
% frames of linear interpolation in (J5_X, J5_Y, r6).
X_target = []; Y_target = []; r6_array = [];

for b = 1:num_boxes
    pX = pick_J5_Xs(b); pY = pick_J5_Ys(b); r6_p = r6_pick_req(b);
    dX = drop_J5_Xs(b); dY = drop_J5_Ys(b); r6_d = r6_drop_req(b);

    % After the last box, return to box 1's pick pose
    if b < num_boxes
        next_pX = pick_J5_Xs(b+1); next_pY = pick_J5_Ys(b+1);
    else
        next_pX = pick_J5_Xs(1);   next_pY = pick_J5_Ys(1);
    end

    r6_retract = 1.0;  % parked slider length while traversing

    % Phase 1: plunge slider down to grab the box at pick
    X_target = [X_target, linspace(pX, pX, frames_per_phase)];
    Y_target = [Y_target, linspace(pY, pY, frames_per_phase)];
    r6_array = [r6_array, linspace(r6_retract, r6_p, frames_per_phase)];

    % Phase 2: retract slider, lifting the box
    X_target = [X_target, linspace(pX, pX, frames_per_phase)];
    Y_target = [Y_target, linspace(pY, pY, frames_per_phase)];
    r6_array = [r6_array, linspace(r6_p, r6_retract, frames_per_phase)];

    % Phase 3: traverse from pick pose to drop pose
    X_target = [X_target, linspace(pX, dX, frames_per_phase)];
    Y_target = [Y_target, linspace(pY, dY, frames_per_phase)];
    r6_array = [r6_array, linspace(r6_retract, r6_retract, frames_per_phase)];

    % Phase 4: plunge slider to release at drop
    X_target = [X_target, linspace(dX, dX, frames_per_phase)];
    Y_target = [Y_target, linspace(dY, dY, frames_per_phase)];
    r6_array = [r6_array, linspace(r6_retract, r6_d, frames_per_phase)];

    % Phase 5: retract slider after release
    X_target = [X_target, linspace(dX, dX, frames_per_phase)];
    Y_target = [Y_target, linspace(dY, dY, frames_per_phase)];
    r6_array = [r6_array, linspace(r6_d, r6_retract, frames_per_phase)];

    % Phase 6: traverse from drop pose back to the next pick pose
    X_target = [X_target, linspace(dX, next_pX, frames_per_phase)];
    Y_target = [Y_target, linspace(dY, next_pY, frames_per_phase)];
    r6_array = [r6_array, linspace(r6_retract, r6_retract, frames_per_phase)];
end
total_frames = length(X_target);

%% Optional factory background image
if isfile('background.png')
    bg_img = imread('background.png');
    has_bg = true;
else
    has_bg = false;
end

%% Figure setup
figure('Name', 'Industrial Pick and Place Cell', 'Position', [100, 100, 1100, 600]);
hold on; axis equal;
axis([-6 20 -2 15]);
title('6-Bar Linkage: Prismatic Joint Grasp');
xlabel('X Position (in)'); ylabel('Y Position (in)');
set(gca, 'Color', [0.91, 0.93, 0.95]);

%% Animation loop
for i = 1:total_frames
    X = X_target(i); Y = Y_target(i); r6 = r6_array(i);

    [t2, t3, t4, t5] = solve_IK(X, Y, r1, r2, r3, r4, r5);
    [X_path, Y_path] = get_coordinates(t2, t3, t5, r1, r2, r3, r5, r6);

    cla;

    % Background
    if has_bg
        imagesc([-6 20], [15 -2], bg_img);
        set(gca, 'YDir', 'normal');
    end

    % Boxes: each is either still at its pick spot, being carried by the
    % gripper during this box's cycle, or already deposited at its drop.
    frames_per_cycle = 6 * frames_per_phase;
    cycle_idx = floor((i-1) / frames_per_cycle) + 1;
    frame_in_cycle = mod(i-1, frames_per_cycle) + 1;

    for b = 1:num_boxes
        if b < cycle_idx
            curr_X = T_drop_X(b); curr_Y = T_drop_Y(b);
        elseif b == cycle_idx
            if frame_in_cycle <= frames_per_phase
                curr_X = T_pick_X(b); curr_Y = T_pick_Y;
            elseif frame_in_cycle > frames_per_phase && frame_in_cycle <= (4 * frames_per_phase)
                curr_X = X_path(4); curr_Y = Y_path(4) - 0.5;
            else
                curr_X = T_drop_X(b); curr_Y = T_drop_Y(b);
            end
        else
            curr_X = T_pick_X(b); curr_Y = T_pick_Y;
        end
        plot(curr_X, curr_Y + 0.5, 's', 'MarkerSize', 30, ...
            'MarkerFaceColor', [0.82, 0.71, 0.55], 'MarkerEdgeColor', 'k');
    end

    % Linkage bars and joints
    plot(X_path([1, 2, 3, 6, 7]), Y_path([1, 2, 3, 6, 7]), ...
        'Color', [0.20, 0.29, 0.37], 'LineStyle', '-', 'LineWidth', 12, ...
        'Marker', 'o', 'MarkerSize', 14, ...
        'MarkerFaceColor', [0.17, 0.24, 0.31], 'MarkerEdgeColor', 'k');

    % Prismatic slider rod
    plot([X_path(3), X_path(4)], [Y_path(3), Y_path(4)], ...
        'Color', [0.95, 0.77, 0.06], 'LineWidth', 8);

    % Gripper fork at the slider tip, oriented along the coupler axis t3
    fork_W = 1.5; fork_L = 1.5;

    dx_perp = -sin(t3) * (fork_W/2); dy_perp =  cos(t3) * (fork_W/2);
    dx_par  =  cos(t3) * fork_L;     dy_par  =  sin(t3) * fork_L;

    F1x = X_path(4) + dx_perp; F1y = Y_path(4) + dy_perp;
    F2x = X_path(4) - dx_perp; F2y = Y_path(4) - dy_perp;
    F3x = F1x + dx_par;        F3y = F1y + dy_par;
    F4x = F2x + dx_par;        F4y = F2y + dy_par;

    plot([F3x, F1x, F2x, F4x], [F3y, F1y, F2y, F4y], ...
        'Color', [0.95, 0.77, 0.06], 'LineWidth', 8);

    drawnow;
end
