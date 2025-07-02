function [polyFunc, polyFunc_dot, polyFunc_ddot] = solvePolynomialSystem(tStart, tEnd, b)
    % Solves the 6x6 linear system and returns polynomial function
    %
    % Inputs:
    %   tStart - scalar value for the first time point
    %   tEnd   - scalar value for the second time point
    %   b      - 6x1 column vector representing the right-hand side
    %
    % Outputs:
    %   polyFunc      - function handle for the 5th order polynomial p(t)
    %   polyFunc_dot  - function handle for the first derivative p'(t)
    %   polyFunc_ddot - function handle for the second derivative p''(t)
    %
    % The function solves the system A*k = b where A is the coefficient matrix
    % based on polynomial evaluation and its derivatives (first and second)
    % at tStart and tEnd.
    
    % Validate inputs
    if ~isscalar(tStart) || ~isscalar(tEnd)
        error('solvePolynomialSystem: tStart and tEnd must be scalar values');
    end
    
    if ~isvector(b) || length(b) ~= 6
        error('solvePolynomialSystem: b must be a 6x1 vector');
    end
    
    % Ensure b is a column vector
    b = b(:);
    
    % Construct the coefficient matrix A
    A = [1,  tStart,  tStart^2,  tStart^3,   tStart^4,    tStart^5;
         0,  1,       2*tStart,  3*tStart^2, 4*tStart^3,  5*tStart^4;
         0,  0,       2,         6*tStart,   12*tStart^2, 20*tStart^3;
         1,  tEnd,    tEnd^2,    tEnd^3,     tEnd^4,      tEnd^5;
         0,  1,       2*tEnd,    3*tEnd^2,   4*tEnd^3,    5*tEnd^4;
         0,  0,       2,         6*tEnd,     12*tEnd^2,   20*tEnd^3];
    
    % Check if the matrix is well-conditioned
    if rcond(A) < eps
        warning('solvePolynomialSystem: Matrix is close to singular. Solution may be inaccurate.');
    end
    
    % Solve the linear system A*k = b
    k = A \ b;

    polyFunc      = @(t) k(1) + k(2)*t + k(3)*t.^2 + k(4)*t.^3   + k(5)*t.^4    + k(6)*t.^5;
    polyFunc_dot  = @(t)        k(2)   + 2*k(3)*t  + 3*k(4)*t.^2 + 4*k(5)*t.^3  + 5*k(6)*t.^4;
    polyFunc_ddot = @(t)                 2*k(3)    + 6*k(4)*t    + 12*k(5)*t.^2 + 20*k(6)*t.^3;

    
    % Display results
    fprintf('\n=============================    Polynomial System Solution    =============================\n');
    fprintf('Time interval: [%.2f, %.2f]\n', tStart, tEnd);
    fprintf('Boundary conditions vector b:\n');
    fprintf('  b = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]''\n', b);
    fprintf('\nSolved polynomial coefficients k:\n');
    fprintf('  k0 = %.4f\n', k(1));
    fprintf('  k1 = %.4f\n', k(2));
    fprintf('  k2 = %.4f\n', k(3));
    fprintf('  k3 = %.4f\n', k(4));
    fprintf('  k4 = %.4f\n', k(5));
    fprintf('  k5 = %.4f\n', k(6));
    fprintf('\nPolynomial: p(t) = %.4f + %.4f*t + %.4f*t^2 + %.4f*t^3 + %.4f*t^4 + %.4f*t^5\n', ...
            k(1), k(2), k(3), k(4), k(5), k(6));
    fprintf('==============================================================================================\n\n');
end
