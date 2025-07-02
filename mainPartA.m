%% Robotics Assignment - Part A - Door Kinematics Analysis
% This script analyzes the kinematics of a door with a rotating handle
% Author: Aristeidis Daskalopoulos
% info: This file includes the whole implementation of assignment's Part A

%% Initialize Problem
clear; clc; close all;
addpath('utils\'); addpath('src\');

% Door parameters
l  = 1.0;    % Door width (m)
h  = 0.7;    % Handle height (m)
lo = 0.1;    % Handle offset from door edge (m)

% Times Values
t0 = 0;  t1 = 2.5;  tf = 5;
ts = 0.05;          % Sampling rate for plots (sec)
t_plot = t0:ts:tf;  % Time vector for plotting [t0, tf]


%% Polynomial Trajectory Planning
[phi1, phi1_dot, phi1_ddot]       =  solvePolynomialSystem(t0, t1, [0; 0; 0; -pi/4; 0; 0]);
[theta1, theta1_dot, theta1_ddot] =  deal(0, 0, 0);  % Constants
[phi2, phi2_dot, phi2_ddot]       =  solvePolynomialSystem(t1, tf, [-pi/4; 0; 0; 0; 0; 0]);
[theta2, theta2_dot, theta2_ddot] =  solvePolynomialSystem(t1, tf, [0; 0; 0; -pi/6; 0; 0]);

% Piecewise function handlers for phi and its derivatives
phi      =  @(t) (t <= t1) .* phi1(t)      +  (t > t1) .* phi2(t);
phi_dot  =  @(t) (t <= t1) .* phi1_dot(t)  +  (t > t1) .* phi2_dot(t);
phi_ddot =  @(t) (t <= t1) .* phi1_ddot(t) +  (t > t1) .* phi2_ddot(t);

% Piecewise function handlers for theta and its derivatives
theta      =  @(t) (t <= t1) .* theta1      +  (t > t1) .* theta2(t);
theta_dot  =  @(t) (t <= t1) .* theta1_dot  +  (t > t1) .* theta2_dot(t);
theta_ddot =  @(t) (t <= t1) .* theta1_ddot +  (t > t1) .* theta2_ddot(t);

plotThetaPhiDerivatives(theta, theta_dot, theta_ddot, phi, phi_dot, phi_ddot, t_plot, t1, t0, tf);


%% Transformation Matrices
% Base frame {0} - Identity transformation
T0 = eye(4);

% Function to compute door frame {D} transformation relative to {0}
T0d = @(t) [
    cos(theta(t)), -sin(theta(t)), 0,  0;
    sin(theta(t)), cos(theta(t)),  0,  2;
    0,             0,              1,  0;
    0,             0,              0,  1;
];

% Function to compute handle frame {H} transformation relative to {D}
Tdh = @(t) [
    0,    cos(phi(t)), -sin(phi(t)),  l - lo;
    -1,   0,           0,             0;
    0,    sin(phi(t)), cos(phi(t)),   h;
    0,    0,           0,             1;
];

% Function to compute handle frame {H} transformation relative to {0}
T0h = @(t) T0d(t) * Tdh(t);


%% Spatial Velocity Analysis of Handle Frame {H}
% Jacobian matrix for handle frame with respect to base frame
% expressed in handle frame coordinates
Jh = @(t) [
    -(l - lo),    0;
    0,            0;
    0,            0;
    0,            1;
    sin(phi(t)),  0;
    cos(phi(t)),  0;
];

% Function to compute the spatial velocity of the handle frame {H}
% with respect to the base frame {0}, expressed in the coordinates of {H}
V0h_H = @(t) Jh(t) * [theta_dot(t); phi_dot(t)];

% Extract rotation matrix and position vector from transformation matrix
R0h = @(t) extractRotation(T0h(t));
p0h = @(t) extractPosition(T0h(t));

% Function to compute the spatial velocity of the handle frame {H}
% with respect to the base frame {0}, expressed in the coordinates of {0}
V0h_0 = @(t) [R0h(t), zeros(3, 3); zeros(3, 3), R0h(t)] * V0h_H(t);

plotSpatialVelocityAnalysis(V0h_0, t_plot, t0, t1, tf);

%% Transformation Matrices for: Initial and Final Positons
plotInitAndFinalPositions(T0, T0d(t0), T0h(t0), T0d(t1), T0h(t1),T0d(tf), T0h(tf));

%% Animation
animateTr3D(T0, T0d, T0h, theta, phi, t_plot);
