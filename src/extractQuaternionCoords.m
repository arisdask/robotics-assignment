function [x, y, z] = extractQuaternionCoords(Q_array)
    % Extract 3D coordinates from quaternion array
    % 
    % info: 
    %   We use the *vector part* of quaternion for unit quaternions, 
    %   to vizualise the orientation trajectory in a 3D plot
    
    x = zeros(1, length(Q_array));
    y = zeros(1, length(Q_array));
    z = zeros(1, length(Q_array));
    
    for i = 1:length(Q_array)
        % Normalize to unit sphere (since we're plotting the orientation only)
        vec_repr = Q_array(i).tovec();
        
        if norm(vec_repr) > 0
            vec_repr = vec_repr / norm(vec_repr);  % Ensure we're on unit sphere
        end
        
        x(i) = vec_repr(1);  y(i) = vec_repr(2);  z(i) = vec_repr(3);
    end
end
