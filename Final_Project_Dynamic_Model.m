clear; close all; clc
%% Global Variables
% from sympy import symbols, diff, simplify, Matrix

syms q1 q2 q3 q4 q5 q6 real
syms dq1 dq2 dq3 dq4 dq5 dq6 real
syms ddq1 ddq2 ddq3 ddq4 ddq5 ddq6 real

syms m1 m2 m3 m4 m5 m6 real       % Link masses
syms l1 l2 l3 l4 l5 l6 real
syms g real 

syms p1 p2 p3 p4 p5 p6

% Joint vectors
q = [q1; q2; q3; q4; q5; q6];
q_d = [dq1; dq2; dq3; dq4; dq5; dq6];
m = [m1; m2; m3; m4; m5; m6];
l = [l1; l2; l3; l4; l5; l6];

% L = [0, 0, 0, 0, 0, 0]; %mm
% m = [3.7, 8.393, 2.33, 1.219, 1.219, 0.1879]; %kg


x = 1.986;
y = 3.117;
phi = 80; %degrees to radians
%% Building a Controller
%{
STEP 1: Derive Dynamic Model
* same for motion and position control but in this case it is for motion
control

M(q)q_dd + C(q,q_d)q_d + g(q) = tau

STEP 2: Select Control Law

PD: tau = Kp*e + Kv*e_d + g(q)

For the catapult project a **PD motion controller** will be determined. This
was decided since the motion trajectory must be smooth in order to launch
the object. This requires controlling the velocity and acceleration which
requires the Proportional controller. It is also important to avoid
overshoot so Derivative controller is ideal. PD controllers are also easier
to tune than PID. 

The Integral (I) is unneccessary for the catapult since it controls the
steady state error. The only time the arm will be in a steady state is
before the launch trajectory and right after launching which are two positions that do
not matter in terms getting the object to the desired location. 

STEP 3: Closed loop control system equation

(d/dt)[q_desired - q; q_d_desired - q_d] = f(q, q_d, q_desired,
q_d_desired, q_dd_desired, M(q), C(q,q_d), g(q))

STEP 4: State-space format
* replace tau from dynamic model with tau in the desired controler

PD: tau = Kp*e + Kv*e_d + g(q)
M(q)q_dd + C(q,q_d)q_d + g(q) = tau

1. Kp*e + Kv*e_d + g(q) = M(q)q_dd + C(q,q_d)q_d + g(q)
2. Kp*e + Kv*e_d = M(q)q_dd + C(q,q_d)q_d

STEP 5: Is the origin in equilibrium
* If e = 0 and q_d = 0 than there is equilibrium at the origin



STEP 6: Stability
STEP 7: Performance
%}
%%
%{
UR5 Model
RRRRRR
Throwing Angles: 30 - 45 degrees

Denavit-Hartenberg (DH)
(simplified trajectory planning for ROS)
Link length (a) – Distance between two consecutive joints.
Link twist (α) – Rotation around the x-axis.
Link offset (d​) – Distance along the z-axis.
Joint angle (θ) – Rotation about the z-axis.

https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
%}

%% STEP 1a: Determine Initial Joint Positions 
% th = [80; -45; -45; -40; -75; 0]; %degrees
% th = [80; -45; -45; -40; 50; 0]; %degrees
th = [80; -45; -45; -40; 0; 0]; %degrees

ur5_model = loadrobot('universalUR5', 'DataFormat', 'column');
show(ur5_model, deg2rad(th));
axis equal;

% Download Peter Corke's Robotics Toolbox UR5 model
% RBE 501 WK 10
figure;
mdl_ur5;
ur5.teach

%% STEP 1b: Determine joint positions using desired theta values

%{ 
DH Parameters [theta, d, a, alpha, offset]
The model provides a DH chart


UR5 [Universal Robotics]:: 6 axis, RRRRRR, stdDH, slowRNE        
+---+-----------+-----------+-----------+-----------+-----------+
| j |     theta |         d |         a |     alpha |    offset |
+---+-----------+-----------+-----------+-----------+-----------+
|  1|         q1|   0.089459|          0|     1.5708|          0|
|  2|         q2|          0|     -0.425|          0|          0|
|  3|         q3|          0|   -0.39225|          0|          0|
|  4|         q4|    0.10915|          0|     1.5708|          0|
|  5|         q5|    0.09465|          0|    -1.5708|          0|
|  6|         q6|     0.0823|          0|          0|          0|
+---+-----------+-----------+-----------+-----------+-----------+
%}

ur5

% Compute the Forward Kinematics using desired joint angles
disp('FK Dynamic Model:')
T = ur5.fkine(th) %FK dynamic model
FK = T.T
disp('Rotation Matrix:')
R = FK(1:3,1:3) % Rotation
disp('Position Matrix:')
P = FK(1:3, 4) % Position

%determine if desired position is in singularity
%non-zero value means no singularity
J_qv = ur5.jacob0(th);

if det(J_qv) == 0
    disp('Singularity')
else 
    disp('Not in Singularity')
end 

x_dot_th1 = diff(FK(1,1),q1)
x_dot_th2 = diff(FK(1,1),q2)
y_dot_th1 = diff(FK(2,1),q1)
y_dot_th2 = diff(FK(2,1),q2)

%% Step 1c: Lagrange
% L = K - P
% Assume trajectory has been defined
q_start = deg2rad([80 -45 -45 -40 0 0]);
q_end   = deg2rad([90 -30 -30 -20 0 0]);
t = linspace(0, 2, 100);
[q_2, qd, qdd] = jtraj(q_start, q_end, t);

% Set dummy dynamic parameters
for i = 1:6
    ur5.links(i).m = 1.0;
    ur5.links(i).r = [0 0 0];
    ur5.links(i).I = [0 0 0 0 0 0];
end

% Evaluate dynamics at timestep k
k = 1;
Q = q(k,:);             % 1×6 row
QD = qd(k,:);
QDD = qdd(k,:);

% Safely compute mass matrix
% M = ur5.inertia(Q);   
% J = ur5.jacob0(Q');  
% J = manip(l1, l2, l3, l4, l5, l6, q1, q2, q3, q4, q5, q6);
% J1 = [Jp1 ; Jo1];
% J2 = [Jp2 ; Jo2];
% J3 = [Jp3 ; Jo3];
% J4 = [Jp4 ; Jo4];
% J5 = [Jp5 ; Jo5];
% J6 = [Jp6 ; Jo6];
syms J1 J2 J3 J4 J5 J6 real
syms v1 v2 v3 v4 v5 v6 real
J = [J1,J2,J3,J4,J5,J6]

v = J * QD';

% Kinetic energy
syms v1 v2 v3 v4 v5 v6 real
v1 = (dq1 * l1);
v2 = (dq2 * l2);
v3 = (dq3 * l3);
v4 = (dq4 * l4);
v5 = (dq5 * l5);
v6 = (dq6 * l6);
% K = 0.5 * QD * M * QD';
K1 = 0.5 * m1 * v1^2;
K2 = 0.5 * m2 * v2^2;
K3 = 0.5 * m3 * v3^2;
K4 = 0.5 * m4 * v4^2;
K5 = 0.5 * m5 * v5^2;
K6 = 0.5 * m6 * v6^2;

K = K1 + K2 + K3 + K4 + K5 + K6

% Potential energy
% z = ur5.fkine(Q').t(3);
% P = 9.81 * 1.0 * z;
% from sympy import sin

p1 = l1 * sin(q1);
p2 = (l1*sin(q1) + l2*sin(q2));
p3 = (l1*sin(q1) + l2*sin(q2) + l3*sin(q3));
p4 = (l1*sin(q1) + l2*sin(q2) + l3*sin(q3) + l4*sin(q4));
p5 = (l1*sin(q1) + l2*sin(q2) + l3*sin(q3) + l4*sin(q4)+ l5*sin(q5));
p6 = (l1*sin(q1) + l2*sin(q2) + l3*sin(q3) + l4*sin(q4)+ l5*sin(q5) + l6*sin(q6));
% Potential energy
P1 = m1 * g * p1;
P2 = m2 * g * p2;
P3 = m3 * g * p3;
P4 = m4 * g * p4;
P5 = m5 * g * p5;
P6 = m6 * g * p6;

P = P1 + P2 + P3 + P4 + P5 + P6

L = K - P

%% Step 1d: Euler - Langrange 
% Plug L into Langrange

% Generalized Force: tau = d/dt(dL/dth_d) - (dL/dth)
% Langrage is calc for the whole system
% Langrage Equation : tau_i = (d/dt)(dL/dq1_dot_i) - (dL/dq_i)
% replace q with:
% th : revolute joint
% d : prismatic joint

% Dynamical Model: tau to allow desired motion % calc for each joint

syms ddq1 ddq2 ddq3 ddq4 ddq5 ddq6 real
q_dd = [ddq1; ddq2; ddq3; ddq4; ddq5; ddq6];
n = 6;

%solve for tau for each joint

t = sym(zeros(n, 1)); 

for i = 1:n
    % dL_dq_d = diff(L, q_d(i))
    dL_dq_d = diff(L, q_d(i));
    %d_dt_dL_dq_d = dL_dq_d(i) * q_d(i) + dL_dq_d(i) * q_dd(i)
    dL_dq = diff(L, q(i))
    
    % Calculate time derivative using the chain rule
    d_dt_dL_dq_d = 0;             % Initialize to zero
    for j = 1:n
        d_dt_dL_dq_d = d_dt_dL_dq_d + diff(dL_dq_d, q(j))*q_d(j); % Chain rule term 1
    end
    d_dt_dL_dq_d = d_dt_dL_dq_d + diff(dL_dq_d, q_d(i))*q_dd(i); % Chain rule term 2
    
    % t(i) = d_dt_dL_dq_d(i) - dL_dq(i)
    t(i) = simplify(d_dt_dL_dq_d - dL_dq);
    
end

tau = t
% M = t(1)/q1_dd
% Joint Torque Vector (q_dd) : [tau1 ; tau2]
% Joint angle acceleration vector : [th1_dd ; th2_dd]

% length from end of link to center of mass
syms Lc1 Lc2 Lc3 Lc4 Lc5 Lc6

% for i = 1:n
%     % Inertial Matrix (M(q)): tau = M * q (q is joint position)
% 
%     %M = [(m1*L1^2) + m2*(L1^2 + 2*L1*L2*c2*L2^2) m2*(L1*L2*c2+L2^2) ; m2*(L1*L2*c2+L2^2) m2*L2^2]
% 
%     % Kinetic Energy equation from Moder Robotics Ch. 8.1.3 video
%     % K = (1/2)*transpose(q_d) * M * q_d
% 
%     % M = (2 * K) / (transpose(q_d) * q_d)
%     [M_sym, rest] = equationsToMatrix(tau, q_dd);
% 
%     % Coriolis/Centripital Coupling Vector (V): V(q,q_dot) = C(q,q_dot) * q_dot
%     % forces and torques that arise due to the interaction between the velocities of different joints in a robotic arm
%     % Centripital Coupling Vector: C = [-2m2l1l2th2_dots2 -m2l1l2]
% 
%     % C = [-m2*L1*L2*s2*(2*th1_d*th2_d*+th2_d) ; m2*L1*L2*th1_d^2*s2]
%     % %assuming center of mass is at the end of the link
% 
%     % C12(i) = 0,  -m(2)*L(1)*Lc2*s2*q2_dot - m(3)*(L(1)*Lc3*s23 + L(2)*Lc3*s23)*q3_dot,  -m(3)*(L(1)*Lc3*s23 + L(2)*Lc3*s23)*q2_dot;
%     % C23(i) = m(2)*L(1)*Lc2*s2*q1_dot,  0, -m(3)*L(2)*Lc3*s23*q3_dot;
%     % C13(i) = m(3)*(L(1)*Lc3*s23 + L(2)*Lc3*s2)*q1_dot,  m(3)*L(2)*Lc3*s2*q2_dot,  0
%     %
%     % C(i) = [ C12(i) ; C23(i)  ; C13(i) ]
% 
% end

syms L1 L2 L3 L4 L5 L6 real
L_links = [L1; L2; L3; L4; L5; L6];

[M, rest] = equationsToMatrix(tau, q_dd);
% Extract Gravity Vector (G)
G = simplify(subs(rest, q_d, zeros(6,1)));
% C = simplify(nonlinear_terms - G);
C = simplify(rest - G); 


% n_DOF Robot (Dynamic model) standard form
% tau = Mq_dd + Cq_d + G

tau = M * q_dd + C + G

% τ=M(q)q¨​+C(q,q˙​)q˙​+g(q)
% HW 3 RBE 501

% syms q1 q2 q3 q1_d q2_d q3_d q1_dd q2_dd q3_dd th1 th2 th3 th1_d th2_d th3_d

% q = [q1;q2;q3];
% q_d = [q1_d; q2_d; q3_d];
% q_dd = [q1_dd; q2_dd; q3_dd];
% 
% f1 = m(1) * g(3);
% f2 = m(2) * g(3);
% f3 = m(3) * g(3);
% 
% F = [f1; f2; f3];

%% Step 1d: Determine desired end effector position than use IK to find desired joint positions
% Inverse Kinematics - Algebraic Method
%
% % disp("Algebraic Solution")
% [th1_alg, th2_alg, th3_alg] = Alg_angle(x,y,L(1,1),L(2,1),L(3,1),phi)

% compute Jacobian matrix
% J = ur5.jacob0([q1 q2 d3 q4 q5 d6])
% 
% q1 = ur5.ikine(FK, 'mask', [1 1 1 0 0 0]);
% q2 = ur5.ikine(FK, 'mask', [1 1 1 0 0 0]);
% q5 = ur5.ikine(FK, 'mask', [0 0 0 1 1 1]);
% q6 = ur5.ikine(FK, 'mask', [0 0 0 1 1 1]);



% pull the end effort x, y, z coordinates from the desired angles
T = ur5.fkine(th);
x = T.t(1)  
y = T.t(2)
z = T.t(3)

% [q1_des, q2_des, q3_des, q4_des, q5_des, q6_des] = invKin(x, y, z);
% 
% figure;
% ur5.plot(q1_des)
% ur5.plot(q2_des)
% ur5.plot(q3_des)
% ur5.plot(q4_des)
% ur5.plot(q5_des)
% ur5.plot(q6_des)

%% STEP 2 - 4: Close the loop with PD Controller

%{ 
τ = M(q)q_dd​+C(q,q_d)q_d​ = (Kp * q_diff) - (Kv * q_d) <- closed loop eq.
PD Gains: Kp and Kv
q_diff = q_desired - q

q_dd = M(q)−1[Kp​(qd​−q)+Kv​(q_d​desired​−q_d​)−C(q,q_d​)q_d​]

(d/dt)[q; q_d] = [q_d; q_ddiff]

State - Space Format: (Optimal Control)
joint-space trajectory
q_l = jtraj(q1_l, q2_l, t);
q_r = jtraj(q1_r, q2_r, t);

joint velocity and acceleration vectors, as a function of time
[q_l, qd, qdd] = jtraj(q1_l, q2_l, t);
[q_r, qd, qdd] = jtraj(q1_r, q2_r, t);

jtraj method of the SerialLink class is more concise then the above step
q_r = right.jtraj(T1, T2, t);
q_l = left.jtraj(T1, T2, t);

Show the trajectory as an animation

WK 8 RBE 502
%}

% State space format
% q_diff = q_desired - q
% 
% q = [q1;q2;q3;q4;q5;q6];
% q_d = [q1_d;q2_d;q3_d;q4_d;q5_d;q6_d];
% q_dd = [q1_dd;q2_dd;q3_dd;q4_dd;q5_dd;q6_dd];
% syms Kp Kv q_desired q_ddesired
syms q_ddesired [6 1] real
syms q_desired [6 1] real
syms Kp Kv [6 1] real
% q_dd = simplify(inv(M) * (Kp * (q_desire - q) + Kv * (q_ddesired - q_d) - C * q_d))
q_dd = simplify( inv(M) * (diag(Kp) * (q_desired - q) + diag(Kv) * (q_ddesired - q_d) - C * q_d - G) );

x_d = [q_d; q_ddiff]

% WK 13 RBE 500
%(2*x_dd) + (8*x_d) + (16*x) = f
% m = 2;
% b = 8;
% k = 16;
% 
% sys = tf([1],[m,b,k]);  % transfer function
% pd = pidtune(sys,'pd'); % pd = Kp + Kd * s
% 
% pidTuner(sys,pd)


%% STEP 5: Is the origin in equilibrium

%{
x1 = 0; x2 = 0; q_diff = 0; q_dot = 0
%}

if (x1 == 0) && (x2 == 0) && (q_diff == 0) && (q_dot == 0)
    disp('Origin is in Equilibrium')
end

%% STEP 6: Stability

%% STEP 7: Performance

%% Plan Out Throws
% Inverse Kinematics function 
% WK 8 RBE 502 Sample Problem Solution

function [q1, q2, q3, q4, q5, q6] = invKin(x, y, z)
a = 0.3;
b = 0.3;
c = 0.3;
line_gamma_y = [0 b*cos(X(end,2))*sin(X(end,1))];
line_gamma_z = [a a+b*sin(X(end,2))];
% Link delta
line_delta_x = [b*cos(X(end,2))*cos(X(end,1)) ee_pos(1,end)];
line_delta_y = [b*cos(X(end,2))*sin(X(end,1)) ee_pos(2,end)];
line_delta_z = [a+b*sin(X(end,2)) ee_pos(3,end)];
figure('Name', 'End Effector Trajectory w/ Stick Model')
plot3(end_eff_x, end_eff_y, end_eff_z, 'g-');
grid on
% Robot Stick Links
line(line_omega_x, line_omega_y, line_omega_z,'Color','magenta');
line(line_alpha_x, line_alpha_y, line_alpha_z,'Color','red');
line(line_beta_x, line_beta_y, line_beta_z,'Color','red','LineStyle','--');
line(line_gamma_x, line_gamma_y, line_gamma_z,'Color','blue');
line(line_delta_x, line_delta_y, line_delta_z,'Color','blue','LineStyle','--');
xlabel('X (meters)');
ylabel('Y (meters)');
legend('End effector trajectory','base link','init link 2',...
'init link 3','final link 2','final link 3');
title('End Effector Trajectory w/ Stick Model With Gcomp');
title('End Effector Trajectory w/ Stick Model Gcomp');
9
r = sqrt(x^2 + y^2 + (z-a)^2);
c_3 = (x^2 + y^2 + (z-a)^2 - b^2 - c^2) / (2 * b * c);
s_3 = abs(sqrt(1 - c_3^2));
q1 = atan2(y,x);
q3 = atan2(s_3,c_3);
q2 = atan2(z-a, sqrt(x^2 + y^2)) - atan2(c*s_3, b+c*c_3);
q4 = 0;
q5 = 0;
q6 = 0;
end

%%
function [th1, th2, th3, th4, th5, th6] = Alg_angle(x, y, L1, L2, L3, L4, L5, L6, phi)
   if (phi >= 0)
        c2 = (x^2 + y^2 - L1^2 - L2^2)/(2*L1*L2);
        s2 = sqrt(1-c2^2);
        %s2_neg = -sqrt(1-c2^2);
        k2 = L2*s2;
        k1 = L1 + (L2*c2);
        % angle calculations
        th1 = round(rad2deg(atan2(y,x)-atan2(k2,k1)));
        th2 = round(rad2deg(atan2(s2,c2)));
        %th2_neg = round(rad2deg(atan2(s2_neg,c2)));
        th3 = round(phi-th1-th2);
        th4 = 0;
        th5 = 0;
        th6 = 0;
   else
        c2 = (x^2 + y^2 - L1^2 - L2^2)/(2*L1*L2);
        %s2 = sqrt(1-c2^2);
        s2_neg = -sqrt(1-c2^2);
        k2 = L2*s2_neg;
        k1 = L1 + (L2*c2);
        % angle calculations
        th1 = round(rad2deg(atan2(y,x)-atan2(k2,k1)));
        %th2 = round(rad2deg(atan2(s2,c2)));
        th2 = round(rad2deg(atan2(s2_neg,c2))); %negative
        th3 = round(phi-th1-th2);
        th4 = 0;
        th5 = 0;
        th6 = 0;
    end
end

function ee = forKin(q1, q2, q3, q4, q5, q6)
a = 0.3;
b = 0.3;
c = 0.3;
rho = b*cos(q2) + c*cos(q2+q3);
x = rho*cos(q1);
y = rho*sin(q1);
z = a + b*sin(q2) + c*sin(q2+q3);
ee = [x;y;z];
end



