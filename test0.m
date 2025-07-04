% Matrix multiplication with trigonometric functions
% Using symbolic variables for theta, phi, l, l_o, and h

% Define symbolic variables
syms theta phi l l_o h

% Define trigonometric functions using symbolic variables
s_theta = sin(theta);
c_theta = cos(theta);
s_phi = sin(phi);
c_phi = cos(phi);

% Define the first matrix A
A = [s_theta,           c_phi*c_theta,      -c_theta*s_phi,     c_theta*(l - l_o);
     -c_theta,          c_phi*s_theta,      -s_phi*s_theta,     s_theta*(l - l_o) + 2;
     0,                 s_phi,              c_phi,              h;
     0,                 0,                  0,                  1];

% Define the second matrix B
B = [0,  0, -1,  0.1;
     0,  1,  0,  0.1;
     1,  0,  0,  0;
     0,  0,  0,  1];

% Perform matrix multiplication
C = A * B;

% Display the matrices
fprintf('Matrix A:\n');
disp(A);

fprintf('\nMatrix B:\n');
disp(B);

fprintf('\nResult C = A * B:\n');
disp(C);

% Simplify the result
C_simplified = simplify(C);
fprintf('\nSimplified result:\n');
disp(C_simplified);
