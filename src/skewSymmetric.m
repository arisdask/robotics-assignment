function S = skewSymmetric(v)
    % Convert 3D vector to skew-symmetric matrix
    % Input:  v - 3x1 vector
    % Output: S - 3x3 skew-symmetric matrix
    
    if length(v) ~= 3
        error('Input vector must be 3-dimensional');
    end
    
    S = [0      -v(3)   v(2);
         v(3)    0     -v(1);
        -v(2)   v(1)    0   ];
end
