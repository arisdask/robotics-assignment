function plotUnitSphere(alphaVal)
    % Plot a unit sphere for reference
    if nargin < 1
        alphaVal = 0.3;  % Default transparency
    end
    
    % Create sphere
    [X, Y, Z] = sphere(30);
    
    % Plot sphere
    surf(X, Y, Z, 'FaceAlpha', alphaVal, 'EdgeAlpha', 0.1, 'FaceColor', [0.8 0.8 0.8]);
    axis equal;
    axis([-1.2 1.2 -1.2 1.2 -1.2 1.2]);
    xlabel('X');  ylabel('Y');  zlabel('Z');
    grid on;      view(-45, 30);
end
