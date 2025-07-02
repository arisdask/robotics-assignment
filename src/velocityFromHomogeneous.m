function vector = velocityFromHomogeneous(V_hat)
    % Extract spatial velocity vector from 4x4 homogeneous representation
    % Input:  V_hat - 4x4 matrix
    % Output: vector - 6x1 spatial velocity [omega; v]
    
    if size(V_hat, 1) ~= 4 || size(V_hat, 2) ~= 4
        error('Input must be 4x4 matrix');
    end
    
    omega_hat = V_hat(1:3, 1:3);
    v = V_hat(1:3, 4);
    
    % Extract angular velocity from skew-symmetric matrix omega_hat
    omega = [omega_hat(3,2); omega_hat(1,3); omega_hat(2,1)];
    
    vector = [omega; v];
end
