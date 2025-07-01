% Load the UR10 model
addpath('robot_model\')
ur10 = ur10robot();

% Define symbolic joint angles
syms q1 q2 q3 q4 q5 q6
q = [q1; q2; q3; q4; q5; q6]; % Symbolic joint angle vector

% Compute the Jacobian in the end-effector frame
J = ur10.jacobe(q);

% Display the Jacobian
disp('Analytical Jacobian in end-effector frame:');
disp(J);

% Simplify the Jacobian (optional, to reduce expression complexity)
J_simplified = simplify(J, 'Steps', 50); % Adjust steps for simplification

% Approximate numerical constants to a specified number of digits
digits = 4; % Number of significant digits for approximation
J_approx = vpa(J_simplified, digits);

% Display the approximated Jacobian
disp('Analytical Jacobian with approximated constants:');
disp(J_approx);
