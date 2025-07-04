% Main script to solve the ODE system
function qr = qrSystemSolver(ur10, qr0, tspan, Vbe_e, h)
    
    % Method 1: Using ODE45 
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', 0.01);
    [t, qr] = ode45(@(t, qr) qrDynamics(t, qr, ur10, Vbe_e), tspan, qr0, options);
    
    % Method 2: Using Euler Method
    % Create time vector for Euler method
    if length(tspan) == 2
        t_euler = tspan(1):h:tspan(2);
    else
        t_euler = tspan;
        h = t_euler(2) - t_euler(1); % Use time step from tspan
    end
    
    % Initialize arrays
    n_steps        =  length(t_euler);
    n_joints       =  length(qr0);
    qr_euler       =  zeros(n_joints, n_steps);
    qr_euler(:, 1) =  qr0(:);
    
    % Euler integration loop
    for i = 1:(n_steps - 1)
        J = ur10.jacobe(qr_euler(:, i));
        Ve = Vbe_e(t_euler(i));
        qr_euler(:, i+1) = qr_euler(:, i) + h * pinv(J) * Ve;
    end
    
    % Plot results
    figure('Name', 'UR10 Robot Joint Angles vs Time', 'NumberTitle', 'off', ...
        'Position', [100, 100, 1300, 650]);
    
    % Plot ODE45 results
    subplot(1, 2, 1);
    plot(t, qr, 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Joint Angles (rad)');
    title('UR10 Robot Joint Angles using ODE45');
    legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'Location', 'best');
    grid on;
    
    % Plot Euler method results
    subplot(1, 2, 2);
    plot(t_euler, qr_euler', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Joint Angles (rad)');
    title('UR10 Robot Joint Angles using Euler Method');
    legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'Location', 'best');
    grid on;
    
    % Calculate and display error
    if length(t) == length(t_euler)
        % Interpolate ODE45 results to match Euler time points for comparison
        qr_ode_interp    =  interp1(t, qr, t_euler, 'linear', 'extrap');
        error_matrix     =  qr_ode_interp - qr_euler';           % Calculate errors at all time points
        rms_error        =  sqrt(mean(error_matrix.^2, 'all'));  % RMS error (Root Mean Square)
        max_error        =  max(abs(error_matrix), [], 'all');   % Max error across all joints and time
        
        fprintf('=================  qrSystemSolver  =================\n');
        fprintf('              +  Methods Comparison  +\n');
        fprintf('RMS Error: %.6f\n', rms_error);
        fprintf('Max Error: %.6f\n', max_error);
        fprintf('=====================================================\n\n');
    end

    qr = qr_euler';  % Returns the Euler Method's Results, as a n x 6 matrix
end

function qr_dot = qrDynamics(t, qr, ur10, Vbe_e)
    % Compute joint velocities: qr_dot = pinv(J(qr)) * Vbe_e(t)
    
    J = ur10.jacobe(qr);
    Ve = Vbe_e(t);
    
    % Compute joint velocities using pseudoinverse
    qr_dot = pinv(J) * Ve;
end
