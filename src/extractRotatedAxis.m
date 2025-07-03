function [x, y, z] = extractRotatedAxis(Q_array)
    % Extracts the rotated body Z-axis over time using Peter Corke's UnitQuaternion class
    n = length(Q_array);
    x = zeros(1, n);
    y = zeros(1, n);
    z = zeros(1, n);

    for i = 1:n
        v_rotated = Q_array(i) * [0; 0; 1];  % Rotate body Z-axis
        x(i) = v_rotated(1);
        y(i) = v_rotated(2);
        z(i) = v_rotated(3);
    end
end
