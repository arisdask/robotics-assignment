% Define symbolic variables
syms phi h l l_o

% Define trigonometric functions
c_phi = cos(phi);
s_phi = sin(phi);

% Define the first matrix
A = [0,      -1,     0;
     c_phi,   0,     s_phi;
     -s_phi,  0,     c_phi];

% Define the second matrix
B = [0,   -h,      0;
     h,    0,     -(l-l_o);
     0,   l-l_o,   0];

% Compute the matrix multiplication
Result = A * B;

% Display the result
disp('Matrix A:')
disp(A)
disp('Matrix B:')
disp(B)
disp('Result A * B:')
disp(Result)

% Simplify the result using trigonometric identities
Result_simplified = simplify(Result, 'Steps', 100);
disp('Simplified Result:')
disp(Result_simplified)

% Apply additional trigonometric simplifications
Result_trig_simplified = simplify(Result_simplified, 'IgnoreAnalyticConstraints', true);
disp('Further trigonometric simplification:')
disp(Result_trig_simplified)