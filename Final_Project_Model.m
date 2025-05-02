%% mdl_planar3_DH_Dynamics_Plot.m
%{

Steps for deriving the end effector trajectory for a 3-link planar robot
  1. Setting up the DH parameters for a 3-link planar robot.
  2. Deriving a simple dynamic model using Lagrange’s method.
  3. Computing the forward kinematics and plotting the arm.

%}

clear; close all; clc;

%% 1. Load the mdl_planar3 Model
% This command (from Peter Corke’s Toolbox) creates a SerialLink object called 'planar3'
mdl_planar3;  
% Configure DH parameters
p3.links(1).d = 0.5;
p3.links(1).alpha = -pi/2;
p3.links(2).a = 0.5;
p3.links(3).a = 0.2;
% grab the link‐parameters into vectors
theta = rad2deg([p3.offset(:)]);
d     = [p3.links.d].';
a     = [p3.links.a].';
alpha = [p3.links.alpha].';
DH = table(theta, d, a, alpha,'VariableNames',{'theta','d','a','alpha'}, 'RowNames', {'L1';'L2';'L3'});
% Display the DH parameters (the SerialLink object stores its DH table in the A property)
disp('DH Parameters for mdl_planar3:');
disp(DH);

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
%{
 For a planar robot (all motion in the x-y plane):
Choose standard DH with:
 - d_i = 0 (no offset along z)
 - alpha_i = 0 (no twist)
 - a_i = link length (L1, L2, L3)
 - theta_i = joint variable (q1, q2, q3)

Construct a 3x4 matrix for the DH parameters:
Each row: [theta, d, a, alpha]
%}
DH = [ q1,   0, L1, pi/2;
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
% Assume that each link’s mass is concentrated at its midpoint.
% Assumed center of mass is halfway along the link.
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

% Kinetic energy for each link:
K1 = (1/2) * m1 * (v1.' * v1);
K2 = (1/2) * m2 * (v2.' * v2);
K3 = (1/2) * m3 * (v3.' * v3);
K_total = simplify(K1 + K2 + K3);

% Potential energy for each link
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
dL_dq  = [diff(Lagr, q1);
           diff(Lagr, q2);
           diff(Lagr, q3)];
       
dL_ddq = [diff(Lagr, dq1);
           diff(Lagr, dq2);
           diff(Lagr, dq3)];
       
% disp('Partial derivative dL/dq:');
% pretty(dL_dq);
% disp('Partial derivative dL/ddq:');
% pretty(dL_ddq);

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
title('3-Link Planar Robot Arm (mdl_planar3)');

% Lable the joints:
text(P0(1), P0(2), '  Base', 'FontSize',12);
text(P1_num(1), P1_num(2), '  Joint1', 'FontSize',12);
text(P2_num(1), P2_num(2), '  Joint2', 'FontSize',12);
text(P3_num(1), P3_num(2), '  EE', 'FontSize',12);

%% 3. Define DH parameters for a generic planar robot
% For a planar 3-link robot (using standard DH conventions):
DH = [ q1, 0, L1, pi/2;
       q2, 0, L2, 0;
       q3, 0, L3, 0];

% Anonymous function for a DH transformation matrix:
DHmatrix = @(theta, d, a, alpha) [ cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
                                    sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                                    0,           sin(alpha),             cos(alpha),            d;
                                    0,           0,                      0,                     1];

%% 4. Forward Kinematics using the mdl_planar3 model
% Define a numeric joint configuration.
% (Remember: Peter Corke’s SerialLink plot() method requires a numeric row vector.)
q_rad = deg2rad([-180, 20, -10]);  % Joint angles in degrees

% Compute the forward kinematics transformation matrix at q_numeric:
T_num = p3.fkine(q_rad);
disp('Forward Kinematics Transformation T (numeric):');
disp(T_num);

%% 5. Plot the Robot
figure;
p3.plot(q_rad);              % Plot the robot at the configuration q
title('Planar 3-Link Robot Arm (mdl_planar3)');
axis equal;

%% 2. Simulation and Control Parameters

% Time span for simulation:
tspan = [0 5];  % in seconds

% Initial joint configuration [q1, q2, q3] (in radians)
% q0 = deg2rad([30, 45, -30]);  
q0 = deg2rad([0, -45, 90]);
% Initial joint velocities:
dq0 = [0, 0, 0];
% Assemble the state vector x = [q1; q2; q3; dq1; dq2; dq3]
x0 = [q0, dq0].';  % convert to a column vector

% Control gains:
Kp = diag([100, 100, 100]);
Kv = diag([20, 20, 20]);

% Desired joint configuration (constant desired position, in radians):
% q_des = deg2rad([pi/2, -pi*2, -45]).';   % column vector (3x1)
q_des = deg2rad([0, -30, 10]).';
dq_des = zeros(3,1);
ddq_des = zeros(3,1); 

% Store gains and desired signals in global variables:
global Kp_global Kv_global q_des_global dq_des_global ddq_des_global
Kp_global = Kp;
Kv_global = Kv;
q_des_global = q_des;
dq_des_global = dq_des;
ddq_des_global = ddq_des;

%% 3. Generate a Joint-Space Trajectory
% Use jtraj to generate a smooth trajectory between q0 and qf.
% Let the movement take 5 seconds with 50 time steps.
% t = linspace(0, 2, 20);
tf = 20;
qf_deg = [45, 45, 45];      % Desired final joint angles (degrees)
qf = deg2rad(qf_deg);       % Convert to radians
qtraj = jtraj(q0, q_des, [0 tf]);   % qtraj is a 50x3 matrix

%% 3. Simulate the Closed-Loop System Using ODE45
[T, X] = ode45(@(t,x)planarArmODE(t,x,p3,2), [0 tf], x0);

%% 4. Plot Joint Trajectories
figure;
plot(T, X(:,1), 'r-', T, X(:,2), 'g-', T, X(:,3), 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Joint Angles (rad)');
legend('q1','q2','q3'); title('Joint Trajectories under PD + Feedforward Control');
grid on;

%% 4. Animate the Robot Moving Along the Trajectory
figure; 
hold on; 

% target location
xt = 2;   % change to your desired X‐coordinate (m)
yt = 0.0;   % change to your desired Y‐coordinate (m)
zt = -3;
% 1) Simple large X marker:
% plot3(xt, yt, zt, 'kx', 'MarkerSize', 25, 'LineWidth', 3);

% OR, if you want a little cross of lines instead of the marker symbol:
L = 0.5;  % length of arm of the cross (m)

% arm along X
plot3([xt-L, xt+L], [yt,    yt   ], [zt, zt], 'k-', 'LineWidth',2);

% arm along Y
plot3([xt,xt ], [yt-L, yt+L], [zt, zt], 'k-', 'LineWidth',2);

% Robot Arm
p3.plot(qtraj,'delay',0.2 );
title('Motion of mdl_planar3 from Initial to Desired Configuration');


%% 5. Compute and Plot End-Effector Trajectory
% Preallocate array for end-effector (EE) positions (x-y in meters)
ee_traj = zeros(length(X), 3);
x = zeros(length(X), 1);
y = zeros(length(X), 1);
z = zeros(length(X), 1);
% ee_des = zeros(size(qtraj)); 
ee_des = zeros(length(X), 3);
for i = 1:size(qtraj,1)
   pos_des = p3.fkine(qtraj(i,:));  
   ee_des(i,:) = pos_des.t; 
end

for i = 1:length(T)
    % Extract the current joint configuration
    % q_current = X(i,1:3)
    % Compute forward kinematics using the SerialLink fkine() method
    % X(i,1:3)
   
    pos = p3.fkine(X(i,1:3));
    pos_trans = pos.t;
    % For a planar robot, extract the x-y coordinates from the translation part (.t)
    x(i) = pos_trans(1);
    y(i) = pos_trans(2);
    z(i) = pos_trans(3);
    ee_traj(i,:) = [x(i);y(i);z(i)];
end

figure;
% plot(ee_traj(:,1), ee_traj(:,2), 'k-', 'LineWidth', 2);
% xlabel('X (m)'); ylabel('Y (m)'); title('End-Effector Trajectory');
% axis equal; grid on;
plot3( x, y, z, 'k-', 'LineWidth', 2 );
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('End-Effector Trajectory (3D)');

figure;
plot(x,z,'g-', 'LineWidth', 2);
xlabel('X (m)');
ylabel('Z (m)');
title('End-Effector Trajectory (XZ plane)');

% 2D planar plot:
figure;
hold on
plot( x, z, 'g-', 'LineWidth', 2 );
plot( ee_des(:,1), ee_des(:,3), 'b-.','LineWidth',2 );
xlabel('X (m)');
ylabel('Z (m)');
title('End-Effector Trajectory (XZ plane)');


%% 6. Launch Projectile
% load('ballPose.mat','ball_at_release','pose');

releaseAngle = qtraj(end,1);     
eeT = p3.fkine(qtraj(end,:));    
releaseX = ee_traj(end,1)
releaseY = ee_traj(end,2);
releaseXv = qtraj(end,1);
releaseYv = qtraj(end,2);


% call your ball functions
% ball_at_release = ball_velocity(releaseAngle, releaseX, releaseY);
% pose = ball_traj(ball_at_release(1),ball_at_release(2),ball_at_release(3),ball_at_release(4),total_time );

% x0  = ball_at_release(1);
% y0  = ball_at_release(2);
% xv0 = ball_at_release(3);
% yv0 = ball_at_release(4);

theta = pi/4;
cup_x = 0.3;
cup_y = 0.3;
cup_radius = 0.01;
cup_front = cup_x - cup_radius;
cup_back = cup_x + cup_radius;

% ball_at_release = [x, y , xv, yv]
ball_at_release = ball_velocity(theta, cup_x, cup_y);
x = ball_at_release(1);
y = ball_at_release(2);
xv = ball_at_release(3);
yv = ball_at_release(4);
total_time = 1;

% pose = ball_traj(x, y, xv, yv, total_time);

% plot(pose(:,1), pose(:,2));  % x vs y
axis equal;
grid on;
hold on;
xlabel('X Position');
ylabel('Z Position');
title('Ball Trajectory');

proj = ball_traj(releaseX, releaseY, releaseXv, releaseYv, 3);

% figure;
plot(proj(:,1), proj(:,2),'r--','LineWidth',2);
% plot(proj(:,1), proj(:,3),'r--','LineWidth',2);
legend('Actual','Desired','Projectile');
% xlabel('X (m)'); ylabel('Y (m)');
title('EE and Projectile Trajectory (XZ Plane)')
% plot3( ballPose(:,1), ballPose(:,2), zeros(size(ballPose,1),1), 'ro-' );

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
        q  = x(1:3);   %  (3x1)
        dq = x(4:6);   %  (3x1)
        
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
        L1 = 1; L2 = 1; L3 = 1;         % whatever your link lengths are
        m1 = 1; m2 = 1; m3 = 1;         % your link masses
        % for each link, put the COM halfway along the x‐axis of that link:
        p3.links(1).m = m1;
        p3.links(1).r = [L1/2, 0, 0];
        p3.links(1).I = [0 0 0];        % neglect rotational inertia for now
        
        p3.links(2).m = m2;
        p3.links(2).r = [L2/2, 0, 0];
        p3.links(2).I = [0 0 0];
        
        p3.links(3).m = m3;
        p3.links(3).r = [L3/2, 0, 0];
        p3.links(3).I = [0 0 0];

        M_val = p3.inertia(q_row);        % Inertia matrix (3x3)
        C_val = p3.coriolis(q_row, dq_row); % Coriolis matrix (3x3)
        G_row = p3.gravload(q_row); 
        G = G_row.'; 

        switch idx 
            case 1
                tau = computeTorque(theta_d, dtheta_d, ddtheta_d, theta, dtheta); %
            
            case 2
                tau = PDplusFeedforwawrd(q,dq,Kp_global,q_des_global,dq_des_global,Kv_global,M_val,ddq_des_global,G);
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
        ddq = pinv(M_val, 1e-6) * (tau - C_val*dq - G);
        % ddq = M_val ./ (tau - C_val*dq - G)
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

    function tau = PDplusFeedforwawrd(q,dq,Kp_global,q_des_global,dq_des_global,Kv_global,M_val,ddq_des_global,G)
        % global Mmatd Cmatd
        % Kp=100*eye(2);
        % Kv=100*eye(2);
        % e=theta_d-theta; % position error
        % de = dtheta_d - dtheta; % velocity error
        % tau= (Kp*e + Kv*de) + Cmatd*dtheta_d + Mmatd*ddtheta_d;

         % Compute the control torque using PD plus feedforward:
        % Control law: tau = Kp*(q_des - q) + Kv*(dq_des - dq) + M(q)*ddq_des.
        % Here ddq_des is zero.
        tau = Kp_global * (q_des_global - q) + Kv_global * (dq_des_global - dq) + M_val * ddq_des_global + G;
    end

    function pose = ball_velocity(theta, cup_x, cup_y)
% returns the pose of the ball at release point
% pose = [x, yv , xv, yv]
%    theta = pi/4;
    robot_release_x = 0;
    robot_release_y = 0.5;
%     cup_x = 10;
%     cup_y = 1;
    g = 9.81;
    a = (sin(theta)*cup_x/cos(theta)) - (cup_y - robot_release_y);
    vel = sqrt((cup_x^2*g)/(2*cos(theta)^2*a));
    x_vel = vel*cos(theta);
    y_vel = vel*sin(theta);
    pose = [robot_release_x, robot_release_y, x_vel, y_vel];
end


function pose = ball_traj(x_i, y_i, xv_i, yv_i,total_time)
    g = -9.81;
    dt = 0.001;
    time = 0:dt:total_time;
    x_pose = [];
    y_pose = [];

    release = false;
    release_time = 0;
    y_release = 0;
    yv_release = 0;

    for i = 1:length(time)
        ti = time(i);
        % robot holding ball 
        % x is EE x pose
        x = xv_i * ti + x_i;

        if ~release && x > 0
            % stores y vel and pose when we release
            release = true;
            release_time = ti;
            y_release = yv_i * ti + y_i;
            yv_release = yv_i;
        end

        % robot holding ball phase
        if ~release
            % if ball is in robot hand
            % y velcoity is ee y velocity
            y = yv_i * ti + y_i;
        else
            % once robot releases at x = 0  ball in free fall
            t_since_release = ti - release_time;
            y = y_release + yv_release * t_since_release + 0.5 * g * t_since_release^2;
        end
        % if ball is on ground
        if y <= 0
            y = 0;
        end
        x_pose = [x_pose; x];
        y_pose = [y_pose; y];
    end
    pose = [x_pose , y_pose];

end

function on_target = tolerance(point, goal, some_tolerance)
    upper = goal + some_tolerance;
    lower = goal - some_tolerance;
    if (lower <= point) && (point <= upper)
        on_target = true;
    else
        on_target = false;
    end
end



