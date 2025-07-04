function plotInitRobotPosition(ur10, q0, T0, T0d, T0h, t0)
    W = [0 2.5 0 2.5 0 2];
    figure('Name', 'Initial Robot Position', 'NumberTitle', 'off', ...
            'Position', [100, 100, 1300, 650]);
    % Plot 1: Door Closed (t0 = 0)
    trplot(T0,     'frame', '0', 'framelabeloffset', [0 -1],   'color', 'k', 'length', 0.3, 'thick', 2);  hold on;
    trplot(T0d(t0), 'frame', 'D', 'framelabeloffset', [0 -0.5], 'color', 'b', 'length', 0.3, 'thick', 2);
    trplot(T0h(t0), 'frame', 'H', 'framelabeloffset', [0, 0.8], 'color', 'r', 'length', 0.3, 'thick', 2);
    
    % Set axis properties
    axis equal; grid on;
    xlim([0, 2.5]);  ylim([0, 2.5]);  zlim([0, 1.5]);
    xlabel('x_0');      ylabel('y_0');      zlabel('z_0');
    ur10.plot(q0, 'workspace', W);
    view(120, 45);
end
