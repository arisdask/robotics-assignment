function plotSpatialVelocityAnalysis(V0h_0, t_plot, t0, t1, tf, h)
    if nargin < 6
        h = 1e-6;  % Default step size for the 'numericalDiff' function
    end
    
    n_points   =  length(t_plot);
    V_data     =  zeros(6, n_points);
    accel_data =  zeros(6, n_points);
    
    % fprintf('Computing velocities and accelerations...\n');
    for i = 1:n_points
        V_data(:, i)     = V0h_0(t_plot(i));
        accel_data(:, i) = numericalDiff(V0h_0, t_plot(i), h);
    end
    
    % Extract components
    v_x = V_data(1, :)';
    v_y = V_data(2, :)';
    v_z = V_data(3, :)';
    omega_x = V_data(4, :)';
    omega_y = V_data(5, :)';
    omega_z = V_data(6, :)';
    
    dv_x = accel_data(1, :)';
    dv_y = accel_data(2, :)';
    dv_z = accel_data(3, :)';
    domega_x = accel_data(4, :)';
    domega_y = accel_data(5, :)';
    domega_z = accel_data(6, :)';
    
    % Generate Plots
    figure('Name', 'Spatial Velocity Analysis of Handle Frame', 'NumberTitle', 'off', ...
        'Position', [200, 50, 1200, 720]);
    
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
    % fprintf('Plotting completed successfully.\n');
    
    
    % Display Summary Information
    fprintf('===========================    Door Kinematics Analysis    ===========================\n');
    fprintf('Time range: [%.2f, %.2f] seconds\n', t0, tf);
    fprintf('Trajectory transition at: %.2f seconds\n', t1);
    fprintf('Total simulation points: %d\n', n_points);
    
    % Calculate norms (magnitudes)
    vNorm       =  sqrt(v_x.^2 + v_y.^2 + v_z.^2);                 % Linear velocity magnitude
    omegaNorm   =  sqrt(omega_x.^2 + omega_y.^2 + omega_z.^2);     % Angular velocity magnitude
    dvNorm      =  sqrt(dv_x.^2 + dv_y.^2 + dv_z.^2);              % Linear acceleration magnitude
    domegaNorm  =  sqrt(domega_x.^2 + domega_y.^2 + domega_z.^2);  % Angular acceleration magnitude
    
    % Find maximum values and their corresponding time indices
    [max_v, idx_v]           =  max(vNorm);
    [max_omega, idx_omega]   =  max(omegaNorm);
    [max_dv, idx_dv]         =  max(dvNorm);
    [max_domega, idx_domega] =  max(domegaNorm);
    
    % Display maximum velocities
    fprintf('\nMaximum velocities and accelerations:\n');
    fprintf('  |v|_max      =  %.3f m/s    at  t = %.2f s\n', max_v,      t_plot(idx_v));
    fprintf('  |ω|_max      =  %.3f rad/s  at  t = %.2f s\n', max_omega,  t_plot(idx_omega));
    fprintf('  |dv/dt|_max  =  %.3f m/s²   at  t = %.2f s\n', max_dv,     t_plot(idx_dv));
    fprintf('  |dω/dt|_max  =  %.3f rad/s² at  t = %.2f s\n', max_domega, t_plot(idx_domega));
    
    % Optional: Display component-wise maxima for detailed analysis
    fprintf('\nComponent-wise maximum analysis:\n');
    fprintf('Linear velocity components:\n');
    fprintf('  max|v_x| = %.3f m/s,        max|v_y| = %.3f m/s,        max|v_z| = %.3f m/s\n', ...
        max(abs(v_x)), max(abs(v_y)), max(abs(v_z)));
    fprintf('Angular velocity components:\n');
    fprintf('  max|ω_x| = %.3f rad/s,      max|ω_y| = %.3f rad/s,      max|ω_z| = %.3f rad/s\n', ...
        max(abs(omega_x)), max(abs(omega_y)), max(abs(omega_z)));
    fprintf('Linear acceleration components:\n');
    fprintf('  max|dv_x/dt| = %.3f m/s²,   max|dv_y/dt| = %.3f m/s²,   max|dv_z/dt| = %.3f m/s²\n', ...
        max(abs(dv_x)), max(abs(dv_y)), max(abs(dv_z)));
    fprintf('Angular acceleration components:\n');
    fprintf('  max|dω_x/dt| = %.3f rad/s², max|dω_y/dt| = %.3f rad/s², max|dω_z/dt| = %.3f rad/s²\n', ...
        max(abs(domega_x)), max(abs(domega_y)), max(abs(domega_z)));
    fprintf('======================================================================================\n\n');
end
