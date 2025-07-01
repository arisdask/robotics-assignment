clear; clc; close all;

% Define symbolic variables
syms t_0 t_1

% Define the 6x6 matrix
M = [1, t_0, t_0^2, t_0^3,   t_0^4,    t_0^5;
     0, 1,   2*t_0, 3*t_0^2, 4*t_0^3,  5*t_0^4;
     0, 0,   2,     6*t_0,   12*t_0^2, 20*t_0^3;
     1, t_1, t_1^2, t_1^3,   t_1^4,    t_1^5;
     0, 1,   2*t_1, 3*t_1^2, 4*t_1^3,  5*t_1^4;
     0, 0,   2,     6*t_1,   12*t_1^2, 20*t_1^3];

% Display the matrix
disp('Matrix M:')
disp(M)

% Calculate the determinant symbolically
det_M = det(M);
disp('Determinant:')
disp(det_M)

% Simplify the determinant
det_M_simplified = simplify(det_M);
disp('Simplified Determinant:')
disp(det_M_simplified)
