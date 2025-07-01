function initAndFinalPositionPlots(T0, T0D_theta0, T0H_theta0, T0D_thetaF, T0H_thetaF)

    % Plot initial and final positions
    figure('Position', [100, 100, 1200, 600]);
    
    % Plot 1: Door closed (theta = 0)
    subplot(1,2,1);
    trplot(T0, 'frame', '0', 'framelabeloffset', [0 -1], 'color', 'k', 'length', 0.3, 'thick', 2);  hold on;
    trplot(T0D_theta0, 'frame', 'D', 'framelabeloffset', [0 -0.5], 'color', 'b', 'length', 0.3, 'thick', 2);
    trplot(T0H_theta0, 'frame', 'H', 'framelabeloffset', [0, 0.8],  'color', 'r', 'length', 0.3, 'thick', 2);
    
    % Set axis properties
    axis equal; grid on;
    xlim([-0.5, 2.5]);  ylim([-0.5, 2.5]);  zlim([0, 1.2]);
    xlabel('x_0');      ylabel('y_0');      zlabel('z_0');
    title('Door Closed (\theta = 0°)');     view(15, 30);
    
    % Plot 2: Door open (theta = -30)
    subplot(1,2,2);
    trplot(T0, 'frame', '0', 'color', 'k', 'length', 0.3, 'thick', 2);  hold on;
    trplot(T0D_thetaF, 'frame', 'D', 'framelabeloffset', [0 -0.5], 'color', 'b', 'length', 0.3, 'thick', 2);
    trplot(T0H_thetaF, 'frame', 'H', 'framelabeloffset', [0, 0.8], 'color', 'r', 'length', 0.3, 'thick', 2);
    
    % Set axis properties
    axis equal; grid on;
    xlim([-0.5, 2.5]);  ylim([-0.5, 2.5]);  zlim([0, 1.2]);
    xlabel('x_0');      ylabel('y_0');      zlabel('z_0');
    title('Door Open (\theta = -30°)');     view(15, 30);
    
    % Adjust Title
    sgtitle('Frames in Initial and Final Positions');
    
    
    % Display transformation matrices
    fprintf('=== Transformation Matrices in Initial and Final Positions ===\n\n');
    
    fprintf('Door Frame {D} - theta = 0°:\n');      disp(T0D_theta0);
    fprintf('Handle Frame {H} - theta = 0°:\n');    disp(T0H_theta0);
    fprintf('Door Frame {D} - theta = -30°:\n');    disp(T0D_thetaF);
    fprintf('Handle Frame {H} - theta = -30°:\n');  disp(T0H_thetaF);
    
    fprintf('==============================================================\n')
end
