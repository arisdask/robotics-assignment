function theta = extractQuaternionAngle(Q_array)
    % Extract theta from quaternion array
    % 
    % info: 
    %   We use the *theta* of quaternion for unit quaternions, 
    %   to vizualise rotation
    
    theta = zeros(1, length(Q_array));
    
    for i = 1:length(Q_array)
        % Normalize to unit sphere (since we're plotting the orientation only)
        [theta(1, i), ~] = Q_array(i).toangvec();
    end
end
