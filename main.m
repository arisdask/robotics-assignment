%% Robotics Assignment - Parts A & B
% Author: Aristeidis Daskalopoulos
% info:   This script includes the whole implementation of Parts A & B

%% Initialize Problem
clear; clc; close all;
addpath('utils\'); addpath('src\');

% Control flag for pause behavior throughout the script
% Pauses are only used for animations to be presented correctly before moving to the next one
debugMode = 1;  % Set the flag to 0 to disable all `pause;` commands

% Door parameters
l  = 1.0;    % Door width (m)
h  = 0.7;    % Handle height (m)
lo = 0.1;    % Handle offset from door edge (m)

% Times Values
t0 = 0;  t1 = 2.5;  tf = 5;
ts = 0.05;          % Sampling rate for plots (sec) (its adjusted to 0.01 at Part B)
t_plot = t0:ts:tf;  % Time vector for plotting [t0, tf]


%% Polynomial Trajectory Planning
fprintf('\nFor φ1 in [t0, t1]:\n');
[phi1, phi1_dot, phi1_ddot]       =  solvePolynomialSystem(t0, t1, [0; 0; 0; -pi/4; 0; 0]);
[theta1, theta1_dot, theta1_ddot] =  deal(0, 0, 0);  % Constants
fprintf('\nFor φ2 in [t1, tf]:\n');
[phi2, phi2_dot, phi2_ddot]       =  solvePolynomialSystem(t1, tf, [-pi/4; 0; 0; 0; 0; 0]);
fprintf('\nFor θ2 in [t1, tf]:\n');
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

%% Quaternion Orientation trajectories on Unit Sphere
plotUnitQuaternions(t_plot, T0h, true);
customPause("Custom Animation", debugMode);

%% Custom Animation
animateTr3D(T0, T0d, T0h, theta, phi, t_plot, 2);
customPause("PartB", debugMode);

%% Part B - Initialize Problem
addpath('robot_model\');  % Load the UR10 model
ur10 = ur10robot();
qr0   = [-1.7752 -1.1823 0.9674 0.2149 1.3664 1.5708];  % initial robot configs

ts     = 0.01;      % Sampling rate for Part B's plots (sec)
t_plot = t0:ts:tf;  % Time vector for plotting [t0, tf]

plotInitRobotPosition(ur10, qr0, T0, T0d, T0h, t0);

%% Transformation Matrices and Spatial Velocity of {e}
The = [  % Function to compute frame {e} transformation relative to {h}
    0, 0, -1,  0.1;
    0, 1,  0,  0.1;
    1, 0,  0,  0;
    0, 0,  0,  1;
];

T0b = [  % Function to compute frame {B} transformation relative to {0}
    1, 0,  0,  1;
    0, 1,  0,  1;
    0, 0,  1,  0;
    0, 0,  0,  1;
];

Tb0 = [  % Function to compute frame {0} transformation relative to {B}
    1, 0,  0,  -1;
    0, 1,  0,  -1;
    0, 0,  1,  0;
    0, 0,  0,  1;
];

Tbe = @(t) Tb0 * T0h(t) * The;  % Function to compute frame {e} transformation relative to {B}

Rbe     = @(t) extractRotation(Tbe(t));
Rbe_dot = @(t) numericalDiff(Rbe, t);
pbe     = @(t) extractPosition(Tbe(t));
pbe_dot = @(t) numericalDiff(pbe, t);

% Function to compute the spatial velocity of the frame {e}
% with respect to the base frame {B}, expressed in the coordinates of {e}
Vbe_e =  @(t) [Rbe(t)' * pbe_dot(t); skewToVec(Rbe(t)' * Rbe_dot(t))];

%% Solve ODE
% Returns the Euler Methods Results
qr     = qrSystemSolver(ur10, qr0, t_plot, Vbe_e);
qr_dot = computeJointVelocities(t_plot, qr);
customPause("Show end effector position and Unit Qauternion", debugMode);

%% End Effector's Posotions & Unit Quaternions, wrt 0 and H
T0e_arraySE3 = ur10.fkine(qr);
T0e_arrayMat = T0e_arraySE3.T;

The_arrayMat = T0e_arrayMat;

for i = 1:length(t_plot)
    The_arrayMat(:, :, i) = pinv( T0h(t_plot(i)) ) * T0e_arrayMat(:, :, i);
end
plotUnitQuaternionsM(t_plot, T0e_arrayMat, true, "g_{0e}");
plotUnitQuaternionsM(t_plot, The_arrayMat, false, "g_{he}");
customPause("Show Final Animation", debugMode);

%% Final Custom Animation 
animate3DWithRobot(ur10, qr, T0, T0d, T0h, theta, phi, t_plot, 10);
