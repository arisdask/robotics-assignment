function plotThetaPhiDerivatives(theta, theta_dot, theta_ddot, phi, phi_dot, phi_ddot, t_plot, t1)
    % Evaluate all functions over time
    phi_values      = phi(t_plot);
    phi_dot_values  = phi_dot(t_plot);
    phi_ddot_values = phi_ddot(t_plot);
    
    theta_values      = theta(t_plot);
    theta_dot_values  = theta_dot(t_plot);
    theta_ddot_values = theta_ddot(t_plot);
    
    % Create figure for PHI functions
    figure('Name', 'Phi Functions and Derivatives', 'NumberTitle', 'off');
    
    % Phi function (position)
    subplot(1, 3, 1);
    plot(t_plot, phi_values, 'b-', 'LineWidth', 2);
    xlabel('Time (t)');
    ylabel('\phi(t)');
    title('\phi(t) - Position');
    grid on;
    hold on;
    plot([t1 t1], [min(phi_values) max(phi_values)], 'r--', 'LineWidth', 1);
    legend('\phi(t)', 'Transition at t_1', 'Location', 'best');
    hold off;
    
    % Phi first derivative (velocity)
    subplot(1, 3, 2);
    plot(t_plot, phi_dot_values, 'b-', 'LineWidth', 2);
    xlabel('Time (t)');
    ylabel('\phi''(t)');
    title('\phi''(t) - First Derivative (Velocity)');
    grid on;
    hold on;
    plot([t1 t1], [min(phi_dot_values) max(phi_dot_values)], 'r--', 'LineWidth', 1);
    legend('\phi''(t)', 'Transition at t_1', 'Location', 'best');
    hold off;
    
    % Phi second derivative (acceleration)
    subplot(1, 3, 3);
    plot(t_plot, phi_ddot_values, 'b-', 'LineWidth', 2);
    xlabel('Time (t)');
    ylabel('\phi''''(t)');
    title('\phi''''(t) - Second Derivative (Acceleration)');
    grid on;
    hold on;
    plot([t1 t1], [min(phi_ddot_values) max(phi_ddot_values)], 'r--', 'LineWidth', 1);
    legend('\phi''''(t)', 'Transition at t_1', 'Location', 'best');
    hold off;
    
    % Adjust subplot spacing for phi figure
    sgtitle('Phi Function and its Derivatives');
    
    % Create figure for THETA functions
    figure('Name', 'Theta Functions and Derivatives', 'NumberTitle', 'off');
    
    % Theta function (position)
    subplot(1, 3, 1);
    plot(t_plot, theta_values, 'g-', 'LineWidth', 2);
    xlabel('Time (t)');
    ylabel('\theta(t)');
    title('\theta(t) - Position');
    grid on;
    hold on;
    plot([t1 t1], [min(theta_values) max(theta_values)], 'r--', 'LineWidth', 1);
    legend('\theta(t)', 'Transition at t_1', 'Location', 'best');
    hold off;
    
    % Theta first derivative (velocity)
    subplot(1, 3, 2);
    plot(t_plot, theta_dot_values, 'g-', 'LineWidth', 2);
    xlabel('Time (t)');
    ylabel('\theta''(t)');
    title('\theta''(t) - First Derivative (Velocity)');
    grid on;
    hold on;
    plot([t1 t1], [min(theta_dot_values) max(theta_dot_values)], 'r--', 'LineWidth', 1);
    legend('\theta''(t)', 'Transition at t_1', 'Location', 'best');
    hold off;
    
    % Theta second derivative (acceleration)
    subplot(1, 3, 3);
    plot(t_plot, theta_ddot_values, 'g-', 'LineWidth', 2);
    xlabel('Time (t)');
    ylabel('\theta''''(t)');
    title('\theta''''(t) - Second Derivative (Acceleration)');
    grid on;
    hold on;
    plot([t1 t1], [min(theta_ddot_values) max(theta_ddot_values)], 'r--', 'LineWidth', 1);
    legend('\theta''''(t)', 'Transition at t_1', 'Location', 'best');
    hold off;
    
    % Adjust subplot spacing for theta figure
    sgtitle('Theta Function and its Derivatives');
    
    % Display some information about the functions
    fprintf('Piecewise functions created:\n');
    fprintf('- phi(t), phi_dot(t), phi_ddot(t)\n');
    fprintf('- theta(t), theta_dot(t), theta_ddot(t)\n');
    fprintf('Transition occurs at t = %.4f\n', t1);
end
