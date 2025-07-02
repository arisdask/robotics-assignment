function plotSpatialVelocityAnalysis(V0h_0, t_plot, t0, t1, tf, h)
    if nargin < 6
        h = 1e-6;  % Default step size
    end
    % Compute Spatial Velocities Over Time
    % Preallocate arrays for efficiency
    n_points = length(t_plot);
    
    % Preallocate as matrices for vectorized operations
    V_data = zeros(6, n_points);
    accel_data = zeros(6, n_points);
    
    % Vectorized computation - much faster than loop
    fprintf('Computing velocities and accelerations...\n');
    for i = 1:n_points
        V_data(:, i) = V0h_0(t_plot(i));
        accel_data(:, i) = numericalDiff(V0h_0, t_plot(i), h);
    end
    
    % Extract components using vectorized indexing
    v_x = V_data(1, :)';
    v_y = V_data(2, :)';
    v_z = V_data(3, :)';
    omega_x = V_data(4, :)';
    omega_y = V_data(5, :)';
    omega_z = V_data(6, :)';
    
    dv_x = accel_data(1, :)';
    dv_y = accel_data(2, :)';
    dv_z = accel_data(3, :)';
    domega_x = accel_data(4, :)';  % Fixed naming: these are angular accelerations
    domega_y = accel_data(5, :)';
    domega_z = accel_data(6, :)';
    
    % Generate Plots
    figure('Position', [100, 200, 1400, 800]);
    
    % Linear velocities
    subplot(2, 2, 1);
    plot(t_plot, [v_x, v_y, v_z], 'LineWidth', 2);
    xline(t1, 'k--', 'LineWidth', 1.5, 'Alpha', 0.7);
    xlabel('Time (s)');
    ylabel('Linear Velocity (m/s)');
    title('Linear Velocities');
    legend({'v_x', 'v_y', 'v_z', 'Transition'}, 'Location', 'best');
    grid on; grid minor;
    xlim([t0, tf]);
    
    % Linear accelerations
    subplot(2, 2, 2);
    plot(t_plot, [dv_x, dv_y, dv_z], 'LineWidth', 2);
    xline(t1, 'k--', 'LineWidth', 1.5, 'Alpha', 0.7);
    xlabel('Time (s)');
    ylabel('Linear Acceleration (m/s²)');
    title('Linear Accelerations');
    legend({'dv_x/dt', 'dv_y/dt', 'dv_z/dt', 'Transition'}, 'Location', 'best');
    grid on; grid minor;
    xlim([t0, tf]);
    
    % Angular velocities
    subplot(2, 2, 3);
    plot(t_plot, [omega_x, omega_y, omega_z], 'LineWidth', 2);
    xline(t1, 'k--', 'LineWidth', 1.5, 'Alpha', 0.7);
    xlabel('Time (s)');
    ylabel('Angular Velocity (rad/s)');
    title('Angular Velocities');
    legend({'\omega_x', '\omega_y', '\omega_z', 'Transition'}, 'Location', 'best');
    grid on; grid minor;
    xlim([t0, tf]);
    
    % Angular accelerations
    subplot(2, 2, 4);
    plot(t_plot, [domega_x, domega_y, domega_z], 'LineWidth', 2);
    xline(t1, 'k--', 'LineWidth', 1.5, 'Alpha', 0.7);
    xlabel('Time (s)');
    ylabel('Angular Acceleration (rad/s²)');
    title('Angular Accelerations');
    legend({'d\omega_x/dt', 'd\omega_y/dt', 'd\omega_z/dt', 'Transition'}, 'Location', 'best');
    grid on; grid minor;
    xlim([t0, tf]);
    
    sgtitle('Spatial Velocity and Acceleration Analysis', 'FontSize', 16, 'FontWeight', 'bold');
    
    fprintf('Plotting completed successfully.\n');
    
    % Display Summary Information
    fprintf('=== Door Kinematics Analysis ===\n');
    fprintf('Time range: %.1f to %.1f seconds\n', t0, tf);
    fprintf('Trajectory transition at: %.1f seconds\n', t1);
    fprintf('Total simulation points: %d\n', n_points);
    
    % Find maximum velocities
    fprintf('\nMaximum velocities:\n');
    fprintf('  |v_x|_max = %.3f m/s at t = %.2f s\n', max(abs(v_x)), t_plot(abs(v_x) == max(abs(v_x))));
    fprintf('  |v_y|_max = %.3f m/s at t = %.2f s\n', max(abs(v_y)), t_plot(abs(v_y) == max(abs(v_y))));
    fprintf('  |v_z|_max = %.3f m/s at t = %.2f s\n', max(abs(v_z)), t_plot(abs(v_z) == max(abs(v_z))));
    fprintf('  |\omega_x|_max = %.3f rad/s at t = %.2f s\n', max(abs(omega_x)), t_plot(abs(omega_x) == max(abs(omega_x))));
    fprintf('  |\omega_y|_max = %.3f rad/s at t = %.2f s\n', max(abs(omega_y)), t_plot(abs(omega_y) == max(abs(omega_y))));
    fprintf('  |\omega_z|_max = %.3f rad/s at t = %.2f s\n', max(abs(omega_z)), t_plot(abs(omega_z) == max(abs(omega_z))));

end
