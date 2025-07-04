function animateTr3D(T0, T0d, T0h, theta, phi, t_plot, speed)
    n_points = length(t_plot);

    figure;
    for i = 1:speed:n_points
        T0d_ti = T0d(t_plot(i));
        T0h_ti = T0h(t_plot(i));
        
        % Plot the frame
        clf;
        trplot(T0,     'frame', '0', 'framelabeloffset', [0 -1],   'color', 'k', 'length', 0.3, 'thick', 2);  hold on;
        trplot(T0d_ti, 'frame', 'D', 'framelabeloffset', [0 -0.5], 'color', 'b', 'length', 0.3, 'thick', 2);
        trplot(T0h_ti, 'frame', 'H', 'framelabeloffset', [0, 0.8], 'color', 'r', 'length', 0.3, 'thick', 2);

        % Set axis properties
        axis equal; grid on;
        xlim([-0.5, 2.5]);  ylim([-0.5, 2.5]);  zlim([0, 1.2]);
        xlabel('x_0');      ylabel('y_0');      zlabel('z_0');
        
        % Dynamic title
        theta_deg = rad2deg(theta(t_plot(i)));
        phi_deg   = rad2deg(phi(t_plot(i)));
        title(sprintf('Time: t = %.2f | Door Angle: \\theta = %.1f° | Handle Angle: \\phi = %.1f°', ...
            t_plot(i), theta_deg, phi_deg));
        view(15, 30);
        drawnow;
    end
end
