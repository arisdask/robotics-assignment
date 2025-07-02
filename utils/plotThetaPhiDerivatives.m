function plotThetaPhiDerivatives(theta, theta_dot, theta_ddot, phi, phi_dot, phi_ddot, t_plot, t1, t0, tf)
    % Evaluate all functions over time
    phi_values      = phi(t_plot);
    phi_dot_values  = phi_dot(t_plot);
    phi_ddot_values = phi_ddot(t_plot);
    
    theta_values      = theta(t_plot);
    theta_dot_values  = theta_dot(t_plot);
    theta_ddot_values = theta_ddot(t_plot);
    
    % Create figure for PHI functions
    figure('Name', 'Phi Functions and Derivatives', 'NumberTitle', 'off', 'Position', [100, 100, 1400, 500]);
    
    % Phi function (position)
    subplot(1, 3, 1);
    plot(t_plot, phi_values, 'b-', 'LineWidth', 2, 'DisplayName', '\phi(t)');
    hold on;
    xline(t1, 'k--', 'LineWidth', 1.5, 'Alpha', 0.7, 'DisplayName', 'Trajectory Transition');
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('\phi(t) (rad)', 'FontSize', 12);
    title('\phi(t) - Position', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 10);
    grid on;
    grid minor;
    xlim([t0, tf]);
    hold off;
    
    % Phi first derivative (velocity)
    subplot(1, 3, 2);
    plot(t_plot, phi_dot_values, 'b-', 'LineWidth', 2, 'DisplayName', 'd\phi/dt');
    hold on;
    xline(t1, 'k--', 'LineWidth', 1.5, 'Alpha', 0.7, 'DisplayName', 'Trajectory Transition');
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('d\phi/dt (rad/s)', 'FontSize', 12);
    title('d\phi/dt - Angular Velocity', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 10);
    grid on;
    grid minor;
    xlim([t0, tf]);
    hold off;
    
    % Phi second derivative (acceleration)
    subplot(1, 3, 3);
    plot(t_plot, phi_ddot_values, 'b-', 'LineWidth', 2, 'DisplayName', 'd²\phi/dt²');
    hold on;
    xline(t1, 'k--', 'LineWidth', 1.5, 'Alpha', 0.7, 'DisplayName', 'Trajectory Transition');
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('d²\phi/dt² (rad/s²)', 'FontSize', 12);
    title('d²\phi/dt² - Angular Acceleration', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 10);
    grid on;
    grid minor;
    xlim([t0, tf]);
    hold off;
    
    % Adjust subplot spacing for phi figure
    sgtitle('Phi Function and its Derivatives', 'FontSize', 16, 'FontWeight', 'bold');
    
    % Create figure for THETA functions
    figure('Name', 'Theta Functions and Derivatives', 'NumberTitle', 'off', 'Position', [100, 200, 1400, 500]);
    
    % Theta function (position)
    subplot(1, 3, 1);
    plot(t_plot, theta_values, 'g-', 'LineWidth', 2, 'DisplayName', '\theta(t)');
    hold on;
    xline(t1, 'k--', 'LineWidth', 1.5, 'Alpha', 0.7, 'DisplayName', 'Trajectory Transition');
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('\theta(t) (rad)', 'FontSize', 12);
    title('\theta(t) - Position', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 10);
    grid on;
    grid minor;
    xlim([t0, tf]);
    hold off;
    
    % Theta first derivative (velocity)
    subplot(1, 3, 2);
    plot(t_plot, theta_dot_values, 'g-', 'LineWidth', 2, 'DisplayName', 'd\theta/dt');
    hold on;
    xline(t1, 'k--', 'LineWidth', 1.5, 'Alpha', 0.7, 'DisplayName', 'Trajectory Transition');
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('d\theta/dt (rad/s)', 'FontSize', 12);
    title('d\theta/dt - Angular Velocity', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 10);
    grid on;
    grid minor;
    xlim([t0, tf]);
    hold off;
    
    % Theta second derivative (acceleration)
    subplot(1, 3, 3);
    plot(t_plot, theta_ddot_values, 'g-', 'LineWidth', 2, 'DisplayName', 'd²\theta/dt²');
    hold on;
    xline(t1, 'k--', 'LineWidth', 1.5, 'Alpha', 0.7, 'DisplayName', 'Trajectory Transition');
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('d²\theta/dt² (rad/s²)', 'FontSize', 12);
    title('d²\theta/dt² - Angular Acceleration', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 10);
    grid on;
    grid minor;
    xlim([t0, tf]);
    hold off;
    
    % Adjust subplot spacing for theta figure
    sgtitle('Theta Function and its Derivatives', 'FontSize', 16, 'FontWeight', 'bold');
end
