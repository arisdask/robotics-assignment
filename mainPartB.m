%% Robotics Assignment - Part Β - Robot Analysis
% This script analyzes the kinematics of a robot opening a door
% Author: Aristeidis Daskalopoulos
% info: This file includes the whole implementation of assignment's Part B

%% Initialize Problem
clear; clc; close all;
addpath('utils\'); addpath('src\');
addpath('robot_model\');  % Load the UR10 model
ur10 = ur10robot();

% Door parameters
l  = 1.0;    % Door width (m)
h  = 0.7;    % Handle height (m)
lo = 0.1;    % Handle offset from door edge (m)

% Times Values
t0 = 0;  t1 = 2.5;  tf = 5;
ts = 0.01;          % Sampling rate for plots (sec)
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

% plotThetaPhiDerivatives(theta, theta_dot, theta_ddot, phi, phi_dot, phi_ddot, t_plot, t1, t0, tf);


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

% plotSpatialVelocityAnalysis(V0h_0, t_plot, t0, t1, tf);


%% Part B stuff...

%% Initial position:
q0 = [-1.7752 -1.1823 0.9674 0.2149 1.3664 1.5708];
W = [0 2.5 0 2.5 0 2];
figure;
% Plot 1: Door Closed (t0 = 0)
trplot(T0,     'frame', '0', 'framelabeloffset', [0 -1],   'color', 'k', 'length', 0.3, 'thick', 2);  hold on;
trplot(T0d(t0), 'frame', 'D', 'framelabeloffset', [0 -0.5], 'color', 'b', 'length', 0.3, 'thick', 2);
trplot(T0h(t0), 'frame', 'H', 'framelabeloffset', [0, 0.8], 'color', 'r', 'length', 0.3, 'thick', 2);

% Set axis properties
axis equal; grid on;
xlim([0, 2.5]);  ylim([0, 2.5]);  zlim([0, 1.5]);
xlabel('x_0');      ylabel('y_0');      zlabel('z_0');     view(15, 30);
ur10.plot(q0, 'workspace', W);
view(120, 45);

%% math stuff

% Function to compute frame {e} transformation relative to {h}
The = [
    0, 0, -1,  0.1;
    0, 1,  0,  0.1;
    1, 0,  0,  0;
    0, 0,  0,  1;
];

% Function to compute frame {B} transformation relative to {0}
T0b = [
    1, 0,  0,  1;
    0, 1,  0,  1;
    0, 0,  1,  0;
    0, 0,  0,  1;
];

% Function to compute frame {0} transformation relative to {B}
Tb0 = [
    1, 0,  0,  -1;
    0, 1,  0,  -1;
    0, 0,  1,  0;
    0, 0,  0,  1;
];

% Function to compute frame {e} transformation relative to {B}
Tbe = @(t) Tb0 * T0h(t) * The;
Rbe = @(t) extractRotation(Tbe(t));
dRbe = @(t) numericalDiff(Rbe, t);
pbe = @(t) extractPosition(Tbe(t));
dpbe = @(t) numericalDiff(pbe, t);

% Function to compute the spatial velocity of the frame {e}
% with respect to the base frame {B}, expressed in the coordinates of {e}
Vbe_e =  @(t) [Rbe(t)' * dpbe(t); vex(Rbe(t)' * dRbe(t))];

[t, qr] = qrSystemDynamics(t_plot, Vbe_e);

%%
%% Initial position:
q0 = [-1.7752 -1.1823 0.9674 0.2149 1.3664 1.5708];
W = [0 2.5 0 2.5 0 2];
figure;
% Plot 1: Door Closed (t0 = 0)
trplot(T0,     'frame', '0', 'framelabeloffset', [0 -1],   'color', 'k', 'length', 0.3, 'thick', 2);  hold on;
trplot(T0d(t0), 'frame', 'D', 'framelabeloffset', [0 -0.5], 'color', 'b', 'length', 0.3, 'thick', 2);
trplot(T0h(t0), 'frame', 'H', 'framelabeloffset', [0, 0.8], 'color', 'r', 'length', 0.3, 'thick', 2);

% Set axis properties
axis equal; grid on;
xlim([0, 2.5]);  ylim([0, 2.5]);  zlim([0, 1.5]);
xlabel('x_0');      ylabel('y_0');      zlabel('z_0');     view(15, 30);
ur10.plot(qr);
view(120, 45);

%% randm 
Tbe_ = ur10.fkine(qr);
phe = zeros(3, 501);
for i = 1:1:501
    p0e = Tbe_(i).t + [1; 1; 0];
    poh = p0h(t_plot(i));
    phe(:, i) = R0h(t_plot(i))' * (p0e - poh);
end
plot(t_plot, phe);

%% Transformation Matrices for: Initial and Final Positons
% plotInitAndFinalPositions(T0, T0d(t0), T0h(t0), T0d(t1), T0h(t1),T0d(tf), T0h(tf));
% 
% % Plot quaternion orientation trajectories on unit sphere
% fprintf('=== 3D Quaternion Trajectory on Unit Sphere ===\n\n');
% T_array = zeros(4, 4, length(t_plot));
% 
% for i = 1:length(t_plot)
%     T_array(:,:,i) = T0h(t_plot(i));
% end
% 
% % Convert array to Unit Quaternions
% Q_array = UnitQuaternion(T_array);
% figure('Name', 'Trajectory', 'Position', [100, 100, 800, 600]);
% 
% % Plot the unit sphere
% plotUnitSphere();
% hold on;
% 
% % Extract quaternion components for 3D plotting
% % Using the vector part of quaternions for 3D coordinates
% % [x, y, z] = extractQuaternionCoords(Q_array);
% [x, y, z] = extractRotatedAxis(Q_array);
% 
% 
% % Plot the trajectory
% plot3(x, y, z, 'r-', 'LineWidth', 2);
% plot3(x(1), y(1), z(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Start
% plot3(x(end), y(end), z(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % End
% 
% title('Quaternion Rotations Trajectory');
% legend('Unit Sphere', 'Trajectory', 'Start', 'End', 'Location', 'best');
% Q_array.animate('fps', 5, 'axis', [-1 1 -1 1 -1 1]);
% 
% 
% %% Animation
% animateTr3D(T0, T0d, T0h, theta, phi, t_plot);
