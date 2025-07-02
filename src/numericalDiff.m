function V_dot = numericalDiff(V_handle, t0, h)
    % V_handle: function handle for 6x1 vector
    % t0: point where you want the derivative
    % h: step size (optional, default: 1e-6)
    
    if nargin < 3
        h = 1e-6;  % Small step size
    end
    
    % Central difference formula: f'(t) ≈ [f(t+h) - f(t-h)] / (2*h)
    V_dot = (V_handle(t0 + h) - V_handle(t0 - h)) / (2*h);
end
