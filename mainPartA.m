%% Robotics Assignment - Part A
% Author: Aristeidis Daskalopoulos
% info: This file includes the whole implemnetation of Part A

%% Initialize Problem
clear; clc; close all;
addpath('utils\'); addpath('src\');

% Door parameters
l  = 1.0;    % Door width (m)
h  = 0.7;    % Handle height (m)
lo = 0.1;    % Handle offset from door edge (m)

%% Define the door angles (in radians)
% Base frame {0} - Identity transformation
T0 = eye(4);

% Function to compute door frame {D} transformation relative to {0}
T0D = @(theta) [cos(theta)   -sin(theta)  0   0;
                 sin(theta)   cos(theta)   0   2;
                 0            0            1   0;
                 0            0            0   1];

% Turn 1: R(z, -90°) around the axes of the moving frame {H} relative to {D}
RDH_z_minus90 = [0   1   0;
                 -1  0   0;
                 0   0   1];

% Turn 2: R(x, φ) around the axes of the moving frame {H} relative to {D}
RDH_x_phi = @(phi) [1   0            0;
                    0   cos(phi)   -sin(phi);
                    0   sin(phi)    cos(phi)];

% The turns defined around the axes of the moving frame {H} relative to {D}
% which means the multiplication will be:
RDH = @(phi) RDH_x_phi(phi) * RDH_z_minus90;

% Function to compute handle frame {H} transformation relative to {D}
% Handle is at (l-l_o, 0, h) in door frame coordinates
TDH = @(phi) [RDH(phi)   [l-lo; 0; h];
              0   0   0   1];

% Function to compute handle frame {H} transformation relative to {0}
T0H = @(theta, phi) T0D(theta) * TDH(phi);

%% Transformation Matrices for: Initial and Final Positons
theta0  =  0;        % Door closed (initial position)
thetaF  =  -pi/6;    % Door open, -30 degrees (final position)
phi0    =  0;        % In both initial and final positon φ = 0

T0D_theta0 = T0D(theta0);
T0H_theta0 = T0H(theta0, phi0);

T0D_thetaF = T0D(thetaF);
T0H_thetaF = T0H(thetaF, phi0);

initAndFinalPositionPlots(T0, T0D_theta0, T0H_theta0, T0D_thetaF, T0H_thetaF);

%% 

