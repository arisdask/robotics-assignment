function qr_dot = computeJointVelocities(t_plot, qr)
    qr_dot = ( qr(2:end, :) - qr(1:end-1, :) ) / ( t_plot(2) - t_plot(1) );
    qr_dot(end+1, :) = qr_dot(end, :);
    
    figure('Name', 'UR10 Robot Joint Velocites vs Time', 'NumberTitle', 'off', ...
            'Position', [100, 100, 700, 500]);
    plot(t_plot, qr_dot, 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel("Joint Angle Velocities (rad/sec)");
    title('UR10 Robot Joint Velocities using Euler Method');
    legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'Location', 'best');
    grid on;
end
