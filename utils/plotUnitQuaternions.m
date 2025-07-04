function plotUnitQuaternions(t_plot, T0h)
    T_array = zeros(4, 4, length(t_plot));
    for i = 1:length(t_plot)
        T_array(:,:,i) = T0h(t_plot(i));
    end
    
    fprintf('=================  plotUnitQuaternions  =================\n');
    mid = ceil(length(t_plot) / 2);
    Qat45deg = UnitQuaternion(T0h(t_plot(mid)));
    fprintf('Example of Corresponding Unit Quaternion Q, at φ=-45 deg:\n');
    fprintf('Qat45deg = %s\n', Qat45deg.char());
    fprintf('Scalar part (s): %.4f\n', Qat45deg.s);
    fprintf('Vector part (v): [%.4f, %.4f, %.4f]\n', Qat45deg.v);
    clear Qat45deg;
    fprintf('========================================================\n');
    
    % Convert array to Unit Quaternions
    Q_array = UnitQuaternion(T_array);
    
    % Plot quaternion orientation trajectories on unit sphere
    figure('Name', '3D Quaternion Trajectory on Unit Sphere', 'NumberTitle', 'off', ...
        'Position', [100, 100, 1300, 650]);
    
    subplot(1, 2, 1);  plotUnitSphere();  hold on;
    % Extract quaternion components for 3D plotting
    % Ploting the vector part, k, of quaternions for 3D coordinates
    [x, y, z] = extractQuaternionCoords(Q_array);
    % Plot the trajectory
    plot3(x, y, z, 'r-', 'LineWidth', 2);
    plot3(x(1), y(1), z(1), 'go', 'MarkerSize', 5, 'MarkerFaceColor', 'g'); % Start 
    scatter3(x(end), y(end), z(end), 100, 'r', 'filled', 'MarkerFaceAlpha', 0.6); % End
    title('Quaternion Orientation Trajectory - Frame H');
    legend('Unit Sphere', 'Trajectory', 'Start', 'End', 'Location', 'best');
    
    subplot(1, 2, 2);  view(-45, 30);  hold on;
    % Extract quaternion components for 3D plotting
    % Ploting the vector part, k, of quaternions for 3D coordinates
    [x, y, z] = extractQuaternionCoords(Q_array);
    % Plot the trajectory
    plot3(x, y, z, 'r-', 'LineWidth', 2);
    plot3(x(1), y(1), z(1), 'go', 'MarkerSize', 5, 'MarkerFaceColor', 'g'); % Start 
    scatter3(x(end), y(end), z(end), 100, 'r', 'filled', 'MarkerFaceAlpha', 0.6); % End
    title('Quaternion Orientation Trajectory (focused) - Frame H');
    legend('Trajectory', 'Start', 'End', 'Location', 'best');
    
    figure('Name', '3D Quaternion Animation on Unit Sphere', 'NumberTitle', 'off', ...
        'Position', [100, 100, 1300, 650]);
    plotUnitSphere();  hold on;
    % Ploting trace of trajectories of x, y, z using Unit Orientations 
    % and animations
    [x, y, z] = extractRotatedAxis(Q_array, 'x');  plot3(x, y, z, 'r-', 'LineWidth', 1);
    [x, y, z] = extractRotatedAxis(Q_array, 'y');  plot3(x, y, z, 'g-', 'LineWidth', 1);
    [x, y, z] = extractRotatedAxis(Q_array, 'z');  plot3(x, y, z, 'b-', 'LineWidth', 1);
    title('Quaternion Animation on Unit Sphere - Frame H');
    legend('Unit Sphere', 'x-axis', 'y-axis', 'z-axis', 'Location', 'best');
    
    Q_array.animate('fps', 5, 'axis', [-1 1 -1 1 -1 1]);
end
