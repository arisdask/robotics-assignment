function V_hat = velocityToHomogeneous(vector)
    % Convert spatial velocity vector to 4x4 homogeneous representation
    % Input:  vector - 6x1 spatial velocity [omega; v]
    % Output: V_hat - 4x4 matrix
    
    if length(vector) ~= 6
        error('Input must be 6-dimensional vector');
    end
    
    omega = vector(1:3);
    v     = vector(4:6);
    
    V_hat = [skewSymmetric(omega), v;
             zeros(1,3),            0];
end
