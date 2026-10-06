function [q1,q2,q3] = inverseKinematics(x_target,y_target,s,q3,p,L_gripper,elbow)

% position on the rail
xc = s*cos(p.theta);
yc = s*sin(p.theta);

% Wrist position needed to place the gripper at target
xw = x_target - L_gripper*cos(q3);
yw = y_target - L_gripper*sin(q3);

% Wrist position relative to rail
X = xw - xc;
Y = yw - yc;

% Distance squared from carriage to wrist
r2 = X^2 + Y^2;

% Calculate relative angle between Link 1 and Link 2
cos_beta = (r2 - p.L1^2 - p.L2^2)/(2*p.L1*p.L2); % Law of Cosine

% Check whether target is reachable
if abs(cos_beta) > 1
    error('Target position is outside the reachable workspace')
end

% Elbow configuration
if elbow == 1
    beta = acos(cos_beta);
else
    beta = -acos(cos_beta);
end

% Link 1 absolute angle
q1 = atan2(Y,X) - atan2(p.L2*sin(beta), p.L1 + p.L2*cos(beta));
% Link 2 absolute angle
q2 = q1 + beta;
end