%% mdl_planar3_DH_Dynamics_Plot.m
% This script demonstrates:
%   1. Setting up the DH parameters for a 3-link planar robot.
%   2. Deriving a simple dynamic model using Lagrange’s method.
%   3. Computing the forward kinematics and plotting the arm.

clear; close all; clc;

%% 1. Load the mdl_planar3 Model
% This command (from Peter Corke’s Toolbox) creates a SerialLink object called 'planar3'
mdl_planar3;  
% Display the DH parameters (the SerialLink object stores its DH table in the A property)
disp('DH Parameters for mdl_planar3:');
disp(p3);

%% 1. Define symbolic variables

% Joint angles (q1, q2, q3) and their derivatives
syms q1 q2 q3 real
syms dq1 dq2 dq3 real
syms ddq1 ddq2 ddq3 real

% Define link lengths (L1, L2, L3) as symbolic parameters
syms L1 L2 L3 real

% Define link masses (m1, m2, m3) and gravity constant g
syms m1 m2 m3 g real

% Define the joint vector and its derivative (for convenience)
q   = [q1; q2; q3];
dq  = [dq1; dq2; dq3];

%% 2. Denavit–Hartenberg Parameters for a 3-Link Planar Robot
% For a planar robot (all motion in the x-y plane):
% Choose standard DH with:
%  - d_i = 0 (no offset along z)
%  - alpha_i = 0 (no twist)
%  - a_i = link length (L1, L2, L3)
%  - theta_i = joint variable (q1, q2, q3)
%
% We construct a 3x4 matrix for the DH parameters:
% Each row: [theta, d, a, alpha]
DH = [ q1,   0, L1, 0;
       q2,   0, L2, 0;
       q3,   0, L3, 0];

%% 3. Forward Kinematics via DH Matrices
% Define an anonymous function that returns the transformation matrix 
% for a given set of DH parameters.
DHmatrix = @(theta, d, a, alpha) [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
                                  sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                                  0,           sin(alpha),             cos(alpha),            d;
                                  0,           0,                      0,                     1];

% Compute the individual transformation matrices:
T01 = DHmatrix(DH(1,1), DH(1,2), DH(1,3), DH(1,4));
T12 = DHmatrix(DH(2,1), DH(2,2), DH(2,3), DH(2,4));
T23 = DHmatrix(DH(3,1), DH(3,2), DH(3,3), DH(3,4));

% The complete transformation from the base to the end-effector:
T02 = simplify(T01 * T12);
T03 = simplify(T02 * T23);

% Extract the end-effector position:
P_end = T03(1:3,4);
disp('Symbolic End-Effector Position:');
pretty(P_end);

%% 4. Dynamic Model via Lagrangian Method
% For simplicity, assume that each link’s mass is concentrated at its midpoint.
% The center of mass for link i is assumed to lie halfway along the link.
% (We ignore rotational inertias for this example.)

% Position of center of mass for each link (in base frame):
P1 = [L1/2*cos(q1);
      L1/2*sin(q1);
      0];
  
P2 = [L1*cos(q1) + L2/2*cos(q1+q2);
      L1*sin(q1) + L2/2*sin(q1+q2);
      0];

P3 = [L1*cos(q1) + L2*cos(q1+q2) + L3/2*cos(q1+q2+q3);
      L1*sin(q1) + L2*sin(q1+q2) + L3/2*sin(q1+q2+q3);
      0];

% Compute the Jacobians for the velocities of each center of mass:
J1 = jacobian(P1, q);
J2 = jacobian(P2, q);
J3 = jacobian(P3, q);

% Compute velocities symbolically:
v1 = simplify(J1 * dq);
v2 = simplify(J2 * dq);
v3 = simplify(J3 * dq);

% Kinetic energy for each link (translational only):
K1 = (1/2) * m1 * (v1.' * v1);
K2 = (1/2) * m2 * (v2.' * v2);
K3 = (1/2) * m3 * (v3.' * v3);
K_total = simplify(K1 + K2 + K3);

% Potential energy for each link (assume gravity acts in negative y direction)
% For a planar arm lying in the horizontal plane, you might set U = 0.
% Here, we assume the arm is vertical so that height is given by the y-coordinate.
U1 = m1 * g * (L1/2 * sin(q1));
U2 = m2 * g * (L1*sin(q1) + L2/2*sin(q1+q2));
U3 = m3 * g * (L1*sin(q1) + L2*sin(q1+q2) + L3/2*sin(q1+q2+q3));
U_total = simplify(U1 + U2 + U3);

% Lagrangian:
Lagr = simplify(K_total - U_total);
disp('Lagrangian:');
pretty(Lagr);

% The equations of motion are given by applying for each joint i:
% d/dt(∂L/∂(dq_i)) - ∂L/∂q_i = tau_i
%
% For a complete derivation, one would differentiate with respect to time.
% Here we simply compute the partial derivatives as a demonstration:
dL_dq  = [diff(Lagr, q1);
           diff(Lagr, q2);
           diff(Lagr, q3)];
       
dL_ddq = [diff(Lagr, dq1);
           diff(Lagr, dq2);
           diff(Lagr, dq3)];
       
disp('Partial derivative dL/dq:');
pretty(dL_dq);
disp('Partial derivative dL/ddq:');
pretty(dL_ddq);

% (A full dynamic model would include time derivatives of dL/ddq to compute tau.)

%% 5. Numerical Substitution and Plotting the Robot Arm
% Provide numerical values for the parameters:
L1_val = 1; L2_val = 1; L3_val = 1;
m1_val = 1; m2_val = 1; m3_val = 1;
g_val  = 9.81;

% Define a sample joint configuration (in radians):
q1_val = deg2rad(30);
q2_val = deg2rad(45);
q3_val = deg2rad(-30);

% Substitute numerical values into the transformation matrices:
T01_num = double(subs(T01, [q1, L1], [q1_val, L1_val]));
T12_num = double(subs(T12, [q2, L2], [q2_val, L2_val]));
T23_num = double(subs(T23, [q3, L3], [q3_val, L3_val]));

% Compute overall forward kinematics numerically:
T03_num = T01_num * T12_num * T23_num;
P_end_num = T03_num(1:3,4);

% Also compute positions of each joint and link-end:
P0 = [0; 0; 0];
P1_num = [L1_val*cos(q1_val); L1_val*sin(q1_val); 0];
P2_num = [L1_val*cos(q1_val) + L2_val*cos(q1_val+q2_val);
          L1_val*sin(q1_val) + L2_val*sin(q1_val+q2_val);
          0];
P3_num = P_end_num;

% Plot the planar robot arm:
figure; hold on; grid on; axis equal;
plot([P0(1) P1_num(1)], [P0(2) P1_num(2)], 'ro-', 'LineWidth', 2);
plot([P1_num(1) P2_num(1)], [P1_num(2) P2_num(2)], 'go-', 'LineWidth', 2);
plot([P2_num(1) P3_num(1)], [P2_num(2) P3_num(2)], 'bo-', 'LineWidth', 2);
xlabel('X (m)'); ylabel('Y (m)');
title('3-Link Planar Robot Arm (mdl\_planar3)');

% Optionally, label the joints:
text(P0(1), P0(2), '  Base', 'FontSize',12);
text(P1_num(1), P1_num(2), '  Joint1', 'FontSize',12);
text(P2_num(1), P2_num(2), '  Joint2', 'FontSize',12);
text(P3_num(1), P3_num(2), '  EE', 'FontSize',12);

%% 3. (Optional) Define DH parameters for a generic planar robot
% For a planar 3-link robot (using standard DH conventions):
DH = [ q1, 0, L1, 0;
       q2, 0, L2, 0;
       q3, 0, L3, 0];

% Anonymous function for a DH transformation matrix:
DHmatrix = @(theta, d, a, alpha) [ cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
                                    sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                                    0,           sin(alpha),             cos(alpha),            d;
                                    0,           0,                      0,                     1];
% (You may use this section to derive symbolic forward kinematics if needed.)

%% 4. Forward Kinematics using the mdl_planar3 model
% Define a numeric joint configuration.
% (Remember: Peter Corke’s SerialLink plot() method requires a numeric row vector.)
q_deg = [30, 45, -30];  % Joint angles in degrees
q_num = deg2rad(q_deg);  % Convert to radians (resulting in a row vector)

% Compute the forward kinematics transformation matrix at q_numeric:
T_num = p3.fkine(q_num);
disp('Forward Kinematics Transformation T (numeric):');
disp(T_num);

%% 5. Plot the Robot
figure;
p3.plot(q_num);              % Plot the robot at the configuration q
title('Planar 3-Link Robot Arm (mdl\_planar3)');
axis equal;

%% 2. Simulation and Control Parameters

% Time span for simulation:
tspan = [0 5];  % in seconds

% Initial joint configuration [q1, q2, q3] (in radians)
q0 = deg2rad([30, 45, -30]);  
% Initial joint velocities (assumed zero):
dq0 = [0, 0, 0];
% Assemble the state vector x = [q1; q2; q3; dq1; dq2; dq3]
x0 = [q0, dq0].';  % convert to a column vector (6x1)

% Control gains:
Kp = diag([100, 100, 100]);
Kv = diag([20, 20, 20]);

% Desired joint configuration (constant desired position, in radians):
q_des = deg2rad([45, 45, 45]).';   % column vector (3x1)
dq_des = zeros(3,1);
ddq_des = zeros(3,1);  % no feedforward acceleration in this example

% Store gains and desired signals in global variables:
global Kp_global Kv_global q_des_global dq_des_global ddq_des_global
Kp_global = Kp;
Kv_global = Kv;
q_des_global = q_des;
dq_des_global = dq_des;
ddq_des_global = ddq_des;

% ODE solver options:
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-6);

%% 3. Generate a Joint-Space Trajectory
% Use jtraj to generate a smooth trajectory between q0 and qf.
% Let the movement take 5 seconds with 50 time steps.
t = linspace(0, 2, 20);
qf_deg = [45, 45, 45];      % Desired final joint angles (degrees)
qf = deg2rad(qf_deg);       % Convert to radians
qtraj = jtraj(q0, qf, t);   % qtraj is a 50x3 matrix

%% 3. Simulate the Closed-Loop System Using ODE45
[t, X] = ode45(@(t,x)planarArmODE(t,x,p3,2), tspan, x0, options);

%% 4. Plot Joint Trajectories
figure;
plot(t, X(:,1), 'r-', t, X(:,2), 'g-', t, X(:,3), 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Joint Angles (rad)');
legend('q1','q2','q3'); title('Joint Trajectories under PD + Feedforward Control');
grid on;

%% 4. Animate the Robot Moving Along the Trajectory
% The plot() method of the SerialLink object can be used to animate the robot.
figure;
p3.plot(qtraj);
title('Motion of mdl\_planar3 from Initial to Desired Configuration');

%% 5. Compute and Plot End-Effector Trajectory
% Preallocate array for end-effector (EE) positions (x-y in meters)
ee_traj = zeros(length(t), 2);
for i = 1:length(t)
    % Extract the current joint configuration (first 3 entries, as a row vector)
    q_current = X(i,1:3);
    % Compute forward kinematics using the SerialLink fkine() method
    T_current = p3.fkine(q_current);
    % For a planar robot, extract the x-y coordinates from the translation part (.t)
    ee_traj(i,:) = T_current.t(1:2).';
end

figure;
plot(ee_traj(:,1), ee_traj(:,2), 'k-', 'LineWidth', 2);
xlabel('X (m)'); ylabel('Y (m)'); title('End-Effector Trajectory');
axis equal; grid on;

%% Functions

function [dx ] = planarArmODE(t,x,p3,idx)
        % theta_d= [w;sin(2*t)]; %You can insert any desired joint trajectory here!
        % dtheta_d =[0; 2*cos(2*t)]; %Time derivative of theta_d
        % ddtheta_d = [0; -4*sin(2*t)]; %Second time derivated of theta_d
        % % theta= x(1:2,1);
        % % dtheta= x(3:4,1);
        % theta= th;
        % dtheta= dth;

            % Global variables used:
        global planar3 Kp_global Kv_global q_des_global dq_des_global ddq_des_global
        % x is a 6x1 vector: [q1; q2; q3; dq1; dq2; dq3]
        
        % Extract current joint angles and velocities:
        q  = x(1:3);   % column vector (3x1)
        dq = x(4:6);   % column vector (3x1)
        
        % Peter Corke’s toolbox methods expect row vectors:
        q_row  = q.';   % 1x3 row vector
        dq_row = dq.';  % 1x3 row vector
    
        
        % global Mmat Cmat Mmatd Cmatd
        % Mmat = [a+2*b*cos(x(2)), d+b*cos(x(2));  d+b*cos(x(2)), d];
        % Cmat = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0];
        % Mmatd = [a+2*b*cos(theta_d(2)), d+b*cos(theta_d(2));  d+b*cos(theta_d(2)), d];
        % Cmatd = [-b*sin(theta_d(2))*dtheta_d(2), -b*sin(theta_d(2))*(dtheta_d(1)+dtheta_d(2)); b*sin(theta_d(2))*dtheta_d(1),0];
        % invM = inv(Mmat);
        % invMC = invM*Cmat;
        % 
        % Obtain the dynamic properties at the current configuration:
        M_val = p3.inertia(q_row);        % Inertia matrix (3x3)
        C_val = p3.coriolis(q_row, dq_row); % Coriolis matrix (3x3)
        G_val = p3.gravload(q_row); 

        switch idx 
            case 1
                tau = computeTorque(theta_d, dtheta_d, ddtheta_d, theta, dtheta); %
            
            case 2
                tau = PDplusFeedforwawrd(q,dq,Kp_global,q_des_global,dq_des_global,Kv_global,M_val,ddq_des_global);
        end
        % torque =[torque, tau];
        % dx=zeros(4,1);
        % dx(1) = x(3);
        % dx(2) = x(4);
        % dx(3:4) = -invMC* x(3:4) +invM*tau; % because ddot theta = -M^{-1}(C \dot Theta) + M^{-1} tau
        
        % Compute the joint accelerations using the dynamic model:
        % M*ddq + C*dq + G = tau   =>   ddq = inv(M) * (tau - C*dq - G')
        % Make sure to transpose G since it is 1x3:
        % ddq = M_val \ (tau - C_val * dq - G_val.');
        ddq = pinv(M_val, 1e-6) * (tau - C_val*dq - G_val.');

        % Assemble the state derivative:
        dx = [dq; ddq];

end

    function tau =computeTorque(theta_d, dtheta_d,ddtheta_d, theta, dtheta)
        global Mmat Cmat
        Kp=100*eye(2);
        Kv=100*eye(2);
        e=theta_d-theta; % position error
        de = dtheta_d - dtheta; % velocity error
        % tau= Mmat*(Kp*e + Kv*de) + Cmat*dtheta + Mmat*ddtheta_d;

       
    end

    function tau = PDplusFeedforwawrd(q,dq,Kp_global,q_des_global,dq_des_global,Kv_global,M_val,ddq_des_global)
        % global Mmatd Cmatd
        % Kp=100*eye(2);
        % Kv=100*eye(2);
        % e=theta_d-theta; % position error
        % de = dtheta_d - dtheta; % velocity error
        % tau= (Kp*e + Kv*de) + Cmatd*dtheta_d + Mmatd*ddtheta_d;

         % Compute the control torque using PD plus feedforward:
        % Control law: tau = Kp*(q_des - q) + Kv*(dq_des - dq) + M(q)*ddq_des.
        % Here ddq_des is zero.
        tau = Kp_global * (q_des_global - q) + Kv_global * (dq_des_global - dq) + M_val * ddq_des_global;
    end



