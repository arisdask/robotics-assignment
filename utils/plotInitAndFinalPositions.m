function plotInitAndFinalPositions(T0, T0D_t0, T0H_t0, T0D_t1, T0H_t1, T0D_tf, T0H_tf)
    % Plot initial and final positions
    figure('Name', 'Initial and Final Positions Plots', 'NumberTitle', 'off', ...
        'Position', [100, 100, 1200, 600]);
    
    % Plot 1: Door Closed (t0 = 0)
    subplot(1,3,1);
    trplot(T0,     'frame', '0', 'framelabeloffset', [0 -1],   'color', 'k', 'length', 0.3, 'thick', 2);  hold on;
    trplot(T0D_t0, 'frame', 'D', 'framelabeloffset', [0 -0.5], 'color', 'b', 'length', 0.3, 'thick', 2);
    trplot(T0H_t0, 'frame', 'H', 'framelabeloffset', [0, 0.8], 'color', 'r', 'length', 0.3, 'thick', 2);
    
    % Set axis properties
    axis equal; grid on;
    xlim([-0.5, 2.5]);  ylim([-0.5, 2.5]);  zlim([0, 1.2]);
    xlabel('x_0');      ylabel('y_0');      zlabel('z_0');
    title('Door Closed (\theta = 0°, \phi = 0°, t_0 = 0s)');     view(15, 30);


    % Plot 2: Door Handle Open (t1 = 2.5s)
    subplot(1,3,2);
    trplot(T0,     'frame', '0', 'framelabeloffset', [0 -1],   'color', 'k', 'length', 0.3, 'thick', 2);  hold on;
    trplot(T0D_t1, 'frame', 'D', 'framelabeloffset', [0 -0.5], 'color', 'b', 'length', 0.3, 'thick', 2);
    trplot(T0H_t1, 'frame', 'H', 'framelabeloffset', [0, 0.8], 'color', 'r', 'length', 0.3, 'thick', 2);
    
    % Set axis properties
    axis equal; grid on;
    xlim([-0.5, 2.5]);  ylim([-0.5, 2.5]);  zlim([0, 1.2]);
    xlabel('x_0');      ylabel('y_0');      zlabel('z_0');
    title('Door Open (\theta = 0°, \phi = -45°, t_1 = 2.5s)');     view(15, 30);

    
    % Plot 3: Door Open (tf = 5s)
    subplot(1,3,3);
    trplot(T0,     'frame', '0', 'framelabeloffset', [0 -1],   'color', 'k', 'length', 0.3, 'thick', 2);  hold on;
    trplot(T0D_tf, 'frame', 'D', 'framelabeloffset', [0 -0.5], 'color', 'b', 'length', 0.3, 'thick', 2);
    trplot(T0H_tf, 'frame', 'H', 'framelabeloffset', [0, 0.8], 'color', 'r', 'length', 0.3, 'thick', 2);
    
    % Set axis properties
    axis equal; grid on;
    xlim([-0.5, 2.5]);  ylim([-0.5, 2.5]);  zlim([0, 1.2]);
    xlabel('x_0');      ylabel('y_0');      zlabel('z_0');
    title('Door Open (\theta = -30°, \phi = 0°, t_f = 5s)');     view(15, 30);
    
    % Adjust Title
    sgtitle('Frames in Initial and Final Positions');
    
    
    % Display transformation matrices
    fprintf('=== Transformation Matrices in Initial and Final Positions ===\n\n');
    
    fprintf('Door Frame {D} - t0 = 0s:\n');      disp(T0D_t0);
    fprintf('Handle Frame {H} - t0 = 0s:\n');    disp(T0H_t0);
    fprintf('Door Frame {D} - t1 = 2.5s:\n');    disp(T0D_t1);
    fprintf('Handle Frame {H} - t1 = 2.5s:\n');  disp(T0H_t1);
    fprintf('Door Frame {D} - tf = 5s:\n');      disp(T0D_tf);
    fprintf('Handle Frame {H} - tf = 5s:\n');    disp(T0H_tf);
    
    fprintf('==============================================================\n')
end
