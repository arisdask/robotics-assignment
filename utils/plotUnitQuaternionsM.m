function plotUnitQuaternionsM(t_plot, T_arrayMat, animateFlag, name)
    x_axis = zeros(length(t_plot), 1);
    y_axis = zeros(length(t_plot), 1);
    z_axis = zeros(length(t_plot), 1);
    for i = 1:length(t_plot)
        T0h_i = T_arrayMat(:, :, i);
        x_axis(i) = T0h_i(1, 4);
        y_axis(i) = T0h_i(2, 4);
        z_axis(i) = T0h_i(3, 4);
    end
    
    % Convert array to Unit Quaternions
    Q_array = UnitQuaternion(T_arrayMat);
    
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
    title(sprintf('3D Quaternion Orientation Trajectory based on %s', name));
    legend('Unit Sphere', 'Trajectory', 'Start', 'End', 'Location', 'best');
    
    subplot(1, 2, 2);  hold on;
    % Extract quaternion components for 3D plotting
    % Ploting the quaternion theta angle, used in quaternions for 3D coordinates
    theta = extractQuaternionAngle(Q_array);
    % Plot the trajectory
    plot(t_plot, theta, 'b-', 'LineWidth', 2);
    title(sprintf('Quaternion Orientation: Angle-Time, based on %s', name));
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('Q.\theta (rad)', 'FontSize', 12);
    grid on;


    figure('Name', 'Frame Position Plot', 'NumberTitle', 'off', ...
        'Position', [200, 100, 800, 500]);
    plot(t_plot, x_axis, 'b-', 'LineWidth', 2);
    hold on;
    plot(t_plot, y_axis, 'r-', 'LineWidth', 2);
    plot(t_plot, z_axis, 'g-', 'LineWidth', 2);
    title(sprintf('End Effector Position Plot (x, y, z), based on %s', name));
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('Position (m)', 'FontSize', 12);
    legend('x-axis', 'y-axis', 'z-axis', 'Location', 'best');
    grid on;
    
    if animateFlag == true
        figure('Name', '3D Quaternion Animation on Unit Sphere', 'NumberTitle', 'off', ...
            'Position', [100, 100, 1300, 650]);
        plotUnitSphere();  hold on;
        [x, y, z] = extractQuaternionRotatedAxis(Q_array, 'x');
        plot3(x, y, z, 'r-', 'LineWidth', 1);
        [x, y, z] = extractQuaternionRotatedAxis(Q_array, 'y');
        plot3(x, y, z, 'g-', 'LineWidth', 1);
        [x, y, z] = extractQuaternionRotatedAxis(Q_array, 'z');
        plot3(x, y, z, 'b-', 'LineWidth', 1);
        title(sprintf('Animation on Unit Sphere based on %s', name));
        legend('Unit Sphere', 'x-axis', 'y-axis', 'z-axis', 'Location', 'best');
        
        Q_array.animate('fps', 5, 'axis', [-1 1 -1 1 -1 1]);
    end
end
