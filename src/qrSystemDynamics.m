% Main script to solve the ODE system
function [t, qr] = qrSystemDynamics(tspan, Vbe_e)
    % Initial conditions
    qr0 = [-1.7752, -1.1823, 0.9674, 0.2149, 1.3664, 1.5708];
    
    % ODE solver options
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', 0.01);
    
    % Solve the ODE system
    [t, qr] = ode45(@(t, qr) qr_dynamics(t, qr, Vbe_e), tspan, qr0, options);
    
    % Plot results
    figure;
    plot(t, qr);
    xlabel('Time (s)');
    ylabel('Joint Angles (rad)');
    title('UR10 Joint Angles vs Time');
    legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'Location', 'best');
    grid on;
    
    % Display final values
    fprintf('Final joint angles: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', qr(end,:));
end

function qr_dot = qr_dynamics(t, qr, Vbe_e)
    ur10 = ur10robot();
    % Compute joint velocities from differential kinematics
    % qr_dot = pinv(J(qr)) * Vbe_e(t)
    
    % Note: You need to replace this with your actual UR10 model
    % This is a placeholder for the Jacobian computation
    J = ur10.jacobe(qr);
    
    % Get end-effector velocity at time t
    Ve = Vbe_e(t);
    
    % Compute joint velocities using pseudoinverse for robustness
    qr_dot = pinv(J) * Ve;
end
