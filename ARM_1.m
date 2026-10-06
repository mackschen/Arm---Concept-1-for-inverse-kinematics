%% Kyler Fourthunders
% Code Robotic Arm Inverse Dynamics
% For a fixed base arm with 2 links and 3 motors to gain
% information for torque needed.
clc;
clear all;
clear;
close all;

% 3-DOF ROBOT ARM
% q(1) = q1   Link 1/ motor angle (rad)
% q(2) = q2   Link 2 angle/ motor (rad)
% q(3) = q3   End-effector rotation (rad)
% All rotation angles are absolute angles measured from the X axis.

% Equation (Robotic Arm Joint Torque Analysis :
% M(q)*qdd + C(q,qd)*qd + G(q) = Q_absolute

% Q = [T1; T2; T3] % Physical motor torques after conversion

% PROPERTIES
p.g = 9.81; %m/s^2

% Link lengths
p.L1 = 0.4452;  % m
p.L2 = 0.4452;  % m

% MASSES Change these with new values ---------------------
p.m0 = 1.4; %  Motor 1/carriage mass
p.m1 = 0.66; % shoulder mass
p.mM2 = 3.9; %  Motor 2 mass (+lum electronics)
p.m2 = 0.58; % elbow mass
p.mM3 = 0.26; % kg - Motor 3 mass
p.m_gripper = 0.24; %gripper section mass
p.m_payload = 6.0; % payload mass + camera
p.m_electronics =0;
p.mM2 = p.m_electronics + p.mM2;
% Mass located at end of Link 2
p.m3 = p.mM3 + p.m_gripper + p.m_payload;

% MOMENT OF INERTIA
% Uniform slender rod assumed for now until we can design more:
% J = (1/12)*m*L^2
% These are about the center of mass of each link.
p.J1 = (1/12)*p.m1*p.L1^2;
p.J2 = (1/12)*p.m2*p.L2^2;
% End-effector rotational inertia
% (TEmp) change this when we know actual geometry from CAD
p.J3 = 0.05; % kg*m^2

p.SF = 2; % saftey factor wanteded
tf = 3; % seconds {Total motion time}

% Time vector
t = linspace(0,tf,500)';
9
% INITIAL CONDITIONS
q1_0 = deg2rad(0);  % Link 1 angle
q2_0 = deg2rad(0);  % Link 2 angle
q3_0 = deg2rad(0);  % End effector angle

% FINAL GRIPPER POSITION
% Click a target using the selection function
% Gripper length, final angle and elbow configuration are set in the function

[x_target,y_target,q1_final,q2_final,q3_final,L_gripper] = selectGripperTarget1(q1_0,q2_0,q3_0,p);

q_initial = [q1_0;
             q2_0; q3_0];
q_final = [q1_final;
           q2_final; q3_final];

% QUINTIC TRAJECTORY
% This trajectory starts and ends with:
% velocity = 0
% acceleration = 0

% h(t) goes from 0 to 1
u = t/tf;
h = 10*u.^3 - 15*u.^4 + 6*u.^5;

% First derivative
hdot = (30*u.^2 - 60*u.^3 + 30*u.^4) / tf;
% Second derivative
hddot = (60*u - 180*u.^2 + 120*u.^3)/tf^2;

N = length(t);
q_des = zeros(N,3);
dq_des = zeros(N,3);
ddq_des = zeros(N,3);

for j = 1:3
    q_des(:,j) = q_initial(j) + (q_final(j) - q_initial(j))*h;
    dq_des(:,j) = (q_final(j) - q_initial(j))*hdot;
    ddq_des(:,j) = (q_final(j) - q_initial(j))*hddot;
end

% INVERSE DYNAMICS
% Motor 1 acts between the base and Link 1
% Motor 2 acts between Link 1 and Link 2
% Motor 3 acts between Link 2 and the gripper
H = [1 1 1;
     0 1 1;
     0 0 1];

Q_required = zeros(N,3);
Q_inertia = zeros(N,3);
Q_velocity = zeros(N,3);
Q_gravity = zeros(N,3);

for i = 1:N
    q = q_des(i,:)';
    dq = dq_des(i,:)';
    ddq = ddq_des(i,:)';
    [M,C,G] = Inverse_Dynamics(q,dq,p);

    % Individual torque contributions
    Q_M = H*(M*ddq);
    Q_C = H*(C*dq);
    Q_G = H*G;

    % Total required actuator torque
    Q = Q_M + Q_C + Q_G;
    Q_required(i,:) = Q';
    Q_inertia(i,:) = Q_M';
    Q_velocity(i,:) = Q_C';
    Q_gravity(i,:) = Q_G';

end

% REQUIRED ACTUATOR LOADS
T1_required = Q_required(:,1);
T2_required = Q_required(:,2);
T3_required = Q_required(:,3);

% MAXIMUM REQUIRED LOADS
T1_max = max(abs(T1_required));
T2_max = max(abs(T2_required));
T3_max = max(abs(T3_required));

% LOADS WITH SAFETY FACTOR
T1_design = T1_max*p.SF;
T2_design = T2_max*p.SF;
T3_design = T3_max*p.SF;

% Results
fprintf('\n')
fprintf('INVERSE DYNAMICS RESULTS\n')
fprintf('\n')
fprintf('Target X = %.3f m\n',x_target)
fprintf('Target Y = %.3f m\n',y_target)
fprintf('\n')
fprintf('Link 1 Absolute Angle = %.2f deg\n',rad2deg(q1_final))
fprintf('Link 2 Absolute Angle = %.2f deg\n',rad2deg(q2_final))
fprintf('Gripper Absolute Angle = %.2f deg\n',rad2deg(q3_final))
fprintf('\nMaximum Required Loads:\n')
fprintf('Motor 1 Torque = %.2f N*m\n',T1_max)
fprintf('Motor 2 Torque = %.2f N*m\n',T2_max)
fprintf('Motor 3 Torque = %.2f N*m\n',T3_max)

fprintf('\nWith Safety Factor of %.2f:\n',p.SF)
fprintf('Design Motor 1 Torque = %.2f N*m\n',T1_design)
fprintf('Design Motor 2 Torque = %.2f N*m\n',T2_design)
fprintf('Design Motor 3 Torque = %.2f N*m\n',T3_design)
plotArmResults(t,q_des,dq_des,ddq_des,Q_required,Q_inertia,Q_velocity,Q_gravity,p,L_gripper)

% ANIMATION
numberRuns = 3;
animateArm(q_des,t,p,numberRuns);