function [x, y, z] = extractQuaternionCoords(Q_array)
    % Extract 3D coordinates from quaternion array
    % This function projects quaternions onto 3D space for visualization
    % 
    % info: 
    %   Using vector part of quaternion for unit quaternions, 
    %   we can use the vector part as 3D coordinates
    
    x = zeros(1, length(Q_array));
    y = zeros(1, length(Q_array));
    z = zeros(1, length(Q_array));
    
    for i = 1:length(Q_array)
        % Normalize to unit sphere (since we're plotting on unit sphere)
        vec_repr = Q_array(i).tovec();  % Use the 3-vector representation
        
        % Ensure we're on unit sphere
        if norm(vec_repr) > 0
            vec_repr = vec_repr / norm(vec_repr);
        end
        
        x(i) = vec_repr(1);
        y(i) = vec_repr(2);
        z(i) = vec_repr(3);
    end
end
