%% Robotics Assignment - Part A - Door Kinematics Analysis
% This script analyzes the kinematics of a door with a rotating handle
% Author: Aristeidis Daskalopoulos
% info: This file includes the whole implementation of Part A


%% Initialize Problem
clear; clc; close all;
addpath('utils\'); addpath('src\');

% Door parameters
l  = 1.0;    % Door width (m)
h  = 0.7;    % Handle height (m)
lo = 0.1;    % Handle offset from door edge (m)

% Times Values
t0 = 0;  t1 = 2.5;  tf = 5;
ts = 0.01;  % Sampling rate for plots (sec)


%% Transformation Matrices
% Base frame {0} - Identity transformation
T0 = eye(4);

% Function to compute door frame {D} transformation relative to {0}
T0d = @(theta) [
    cos(theta),   -sin(theta),  0,   0;
    sin(theta),   cos(theta),   0,   2;
    0,            0,            1,   0;
    0,            0,            0,   1
];

% Function to compute handle frame {H} transformation relative to {D}
% Handle is at (l-l_o, 0, h) in door frame coordinates
Tdh = @(phi) [
    0,    cos(phi),   -sin(phi),   l - lo;
    -1,   0,          0,           0;
    0,    sin(phi),   cos(phi),    h;
    0,    0,          0,           1
];

% Function to compute handle frame {H} transformation relative to {0}
T0h = @(theta, phi) T0d(theta) * Tdh(phi);


%% Spatial Velocity Analysis of Handle Frame {H}
% Jacobian matrix for handle frame with respect to base frame
% expressed in handle frame coordinates
Jh = @(theta, phi) [
    -(l - lo),    0;
    0,            0;
    0,            0;
    0,            1;
    sin(phi),     0;
    cos(phi),     0;
];

% Function to compute the spatial velocity of the handle frame {H} 
% with respect to the base frame {0}, **expressed** in the coordinates of frame {H}
V0h_H = @(theta, phi, theta_dot, phi_dot) Jh(theta, phi) * [theta_dot; phi_dot];

% Adjoint transformation matrix (Γ) for coordinate frame conversion
% Converts spatial velocity from handle frame coordinates to base frame coordinates
adjointTransformation = @(T) [
    T(1:3, 1:3),  skewSymmetric(T(1:3, 4)) * T(1:3, 1:3);
    zeros(3,3),   T(1:3, 1:3);
];

% Function to compute the spatial velocity of the handle frame {H} 
% with respect to the base frame {0}, **expressed** in the coordinates of frame {0}
V0h_0 = @(theta, phi, theta_dot, phi_dot) ...
    adjointTransformation(T0h(theta, phi)) * V0h_H(theta, phi, theta_dot, phi_dot);


%% Polynomial Trajectory Planning
[phi1, phi1_dot, phi1_ddot]       =  solvePolynomialSystem(t0, t1, [0; 0; 0; -pi/4; 0; 0]);
[theta1, theta1_dot, theta1_ddot] =  deal(0, 0, 0);  % Constants
[phi2, phi2_dot, phi2_ddot]       =  solvePolynomialSystem(t1, tf, [-pi/4; 0; 0; 0; 0; 0]);
[theta2, theta2_dot, theta2_ddot] =  solvePolynomialSystem(t1, tf, [0; 0; 0; -pi/6; 0; 0]);

% Create piecewise function handlers for phi and its derivatives
phi      =  @(t) (t <= t1) .* phi1(t)      +  (t > t1) .* phi2(t);
phi_dot  =  @(t) (t <= t1) .* phi1_dot(t)  +  (t > t1) .* phi2_dot(t);
phi_ddot =  @(t) (t <= t1) .* phi1_ddot(t) +  (t > t1) .* phi2_ddot(t);

% Create piecewise function handlers for theta and its derivatives
theta      =  @(t) (t <= t1) .* theta1      +  (t > t1) .* theta2(t);
theta_dot  =  @(t) (t <= t1) .* theta1_dot  +  (t > t1) .* theta2_dot(t);
theta_ddot =  @(t) (t <= t1) .* theta1_ddot +  (t > t1) .* theta2_ddot(t);


% Create time vector for plotting
t_plot = t0:ts:tf;
plotThetaPhiDerivatives(theta, theta_dot, theta_ddot, phi, phi_dot, phi_ddot, t_plot, t1);


%% Compute Spatial Velocities Over Time
% Preallocate arrays for efficiency
n_points = length(t_plot);
v_x = zeros(n_points, 1);
v_y = zeros(n_points, 1);
v_z = zeros(n_points, 1);
omega_x = zeros(n_points, 1);
omega_y = zeros(n_points, 1);
omega_z = zeros(n_points, 1);

% Compute spatial velocity at each time point
for i = 1:n_points
    t_current = t_plot(i);
    
    % Get current joint angles and velocities
    theta_current     =  theta(t_current);
    phi_current       =  phi(t_current);
    theta_dot_current =  theta_dot(t_current);
    phi_dot_current   =  phi_dot(t_current);
    
    % Compute spatial velocity in base frame coordinates
    V_current = V0h_0(theta_current, phi_current, theta_dot_current, phi_dot_current);
    
    % Extract linear and angular velocity components
    v_x(i) = V_current(1);
    v_y(i) = V_current(2);
    v_z(i) = V_current(3);
    omega_x(i) = V_current(4);
    omega_y(i) = V_current(5);
    omega_z(i) = V_current(6);
end

%% Generate Plots
% Plot 1: Linear Velocities
figure;
clf;
plot(t_plot, v_x, 'r-', 'LineWidth', 2, 'DisplayName', 'v_x');
hold on;
plot(t_plot, v_y, 'g-', 'LineWidth', 2, 'DisplayName', 'v_y');
plot(t_plot, v_z, 'b-', 'LineWidth', 2, 'DisplayName', 'v_z');
hold off;

% Add vertical line at t1 to show trajectory transition
xline(t1, 'k--', 'LineWidth', 1.5, 'Alpha', 0.7, 'DisplayName', 'Trajectory Transition');

xlabel('Time (s)', 'FontSize', 12);
ylabel('Linear Velocity (m/s)', 'FontSize', 12);
title('Handle Linear Velocities in Base Frame', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
grid on;
grid minor;
xlim([t0, tf]);

% Plot 2: Angular Velocities
figure;
clf;
plot(t_plot, omega_x, 'r-', 'LineWidth', 2, 'DisplayName', '\omega_x');
hold on;
plot(t_plot, omega_y, 'g-', 'LineWidth', 2, 'DisplayName', '\omega_y');
plot(t_plot, omega_z, 'b-', 'LineWidth', 2, 'DisplayName', '\omega_z');
hold off;

% Add vertical line at t1 to show trajectory transition
xline(t1, 'k--', 'LineWidth', 1.5, 'Alpha', 0.7, 'DisplayName', 'Trajectory Transition');

xlabel('Time (s)', 'FontSize', 12);
ylabel('Angular Velocity (rad/s)', 'FontSize', 12);
title('Handle Angular Velocities in Base Frame', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
grid on;
grid minor;
xlim([t0, tf]);

%% Display Summary Information
fprintf('=== Door Kinematics Analysis ===\n');
fprintf('Time range: %.1f to %.1f seconds\n', t0, tf);
fprintf('Trajectory transition at: %.1f seconds\n', t1);
fprintf('Total simulation points: %d\n', n_points);

% Find maximum velocities
fprintf('\nMaximum velocities:\n');
fprintf('  |v_x|_max = %.3f m/s at t = %.2f s\n', max(abs(v_x)), t_plot(abs(v_x) == max(abs(v_x))));
fprintf('  |v_y|_max = %.3f m/s at t = %.2f s\n', max(abs(v_y)), t_plot(abs(v_y) == max(abs(v_y))));
fprintf('  |v_z|_max = %.3f m/s at t = %.2f s\n', max(abs(v_z)), t_plot(abs(v_z) == max(abs(v_z))));
fprintf('  |\omega_x|_max = %.3f rad/s at t = %.2f s\n', max(abs(omega_x)), t_plot(abs(omega_x) == max(abs(omega_x))));
fprintf('  |\omega_y|_max = %.3f rad/s at t = %.2f s\n', max(abs(omega_y)), t_plot(abs(omega_y) == max(abs(omega_y))));
fprintf('  |\omega_z|_max = %.3f rad/s at t = %.2f s\n', max(abs(omega_z)), t_plot(abs(omega_z) == max(abs(omega_z))));


%% Transformation Matrices for: Initial and Final Positons
theta0  =  0;        % Door closed (initial position)
thetaF  =  -pi/6;    % Door open, -30 degrees (final position)
phi0    =  0;        % In both initial and final positon φ = 0

T0D_theta0 = T0d(theta0);
T0H_theta0 = T0h(theta0, phi0);

T0D_thetaF = T0d(thetaF);
T0H_thetaF = T0h(thetaF, phi0);

initAndFinalPositionPlots(T0, T0D_theta0, T0H_theta0, T0D_thetaF, T0H_thetaF);

%% Animation

figure;
for i = 1:n_points
    clf;  % Correct: clears current figure
    
    % Compute transformation matrix
    T0h_ = T0h(theta(t_plot(i)), phi(t_plot(i)));
    
    % Plot the frame
    trplot(T0h_, 'frame', 'H', 'framelabeloffset', [0, 0.8], ...
           'color', 'r', 'length', 0.3, 'thick', 2);
    
    % Set axis properties
    axis equal; grid on;
    xlim([-0.5, 2.5]); ylim([-0.5, 2.5]); zlim([0, 1.2]);
    xlabel('x_0'); ylabel('y_0'); zlabel('z_0');
    
    % Dynamic title
    theta_deg = rad2deg(theta(t_plot(i)));
    title(sprintf('Door Angle: \\theta = %.1f°', theta_deg));
    
    view(15, 30);
    
    drawnow;  % Force MATLAB to update the figure
end
