function [x, y, z] = extractQuaternionRotatedAxis(Q_array, axis)
    v = [0; 0; 1];
    if axis == 'x'
        v = [1; 0; 0];
    elseif axis == 'y'
        v = [0; 1; 0];
    end
        
    % Extracts the rotated body 'axis'-axis over time
    n = length(Q_array);
    x = zeros(1, n);  y = zeros(1, n);  z = zeros(1, n);

    for i = 1:n
        v_rotated = Q_array(i) * v;  % Rotate body 'axis'-axis
        x(i) = v_rotated(1);  y(i) = v_rotated(2);  z(i) = v_rotated(3);
    end
end
