% Define symbolic variables
syms theta phi l l_o h theta_dot phi_dot

% Define cos and sin shortcuts for simplification
c_theta = cos(theta);
s_theta = sin(theta);
c_phi = cos(phi);
s_phi = sin(phi);

% Define rotation matrix R_{0h}
R_0h = [s_theta,           c_phi*c_theta,       -c_theta*s_phi;
        -c_theta,          c_phi*s_theta,       -s_phi*s_theta;
        0,                 s_phi,               c_phi];

% Define position vector p0h
p0h = [c_theta*(l - l_o);
       s_theta*(l - l_o) + 2;
       h;];

% Define skew-symmetric matrix p0h_hat
p0h_hat = [0,                         -h,                 s_theta*(l - l_o) + 2;
           h,                         0,                  -c_theta*(l - l_o);
           -s_theta*(l - l_o) + 2,    c_theta*(l - l_o),  0];

% Define velocity vector v_c
v_c = [-(l - l_o)*theta_dot;
       0;
       0];

% Define angular velocity vector omega
omega = [phi_dot;
         s_phi*theta_dot;
         c_phi*theta_dot];

% Calculate the final answer
Ans = R_0h * v_c + p0h_hat * R_0h * omega;

% Simplify the result
Ans_simplified = simplify(Ans);

disp('Simplified Answer:');
disp(Ans_simplified);
