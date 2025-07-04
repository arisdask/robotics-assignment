function F_dot = numericalDiff(F_handle, t0, h)
    % Inputs:
    %   F_handle - function handle for Nx1 vector function F(t)
    %   t0       - point where we want the derivative
    %   h        - step size (optional, default: 1e-6)
    
    if nargin < 3
        h = 1e-6;
    end
    
    % Central difference formula: F'(t) = [F(t+h) - F(t-h)] / (2*h)
    F_dot = (F_handle(t0 + h) - F_handle(t0 - h)) / (2*h);
end
