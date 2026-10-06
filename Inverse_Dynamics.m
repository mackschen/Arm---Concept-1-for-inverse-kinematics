function [M,C,G] = Inverse_Dynamics(q,dq,p)
% For fixed base, 2 links and 3 motors

% coordinates
q1 = q(1);
q2 = q(2);
q3 = q(3);

% velocities
dq1 = dq(1);
dq2 = dq(2);
dq3 = dq(3);

% Mass matrix constants
A = p.J1 + p.L1^2*(p.m1/4 + p.mM2 + p.m2 + p.m3);
B = p.J2 + p.L2^2*(p.m2/4 + p.m3);
D = p.L1*p.L2*(p.m2/2 + p.m3);

% MASS MATRIX
M = zeros(3,3);
M(1,1) = A;
M(1,2) = D*cos(q1-q2);
M(2,1) = M(1,2);
M(2,2) = B;
M(3,3) = p.J3;

% CORIOLIS / CENTRIFUGAL MATRIX
C = zeros(3,3);
C(1,2) = D*sin(q1-q2)*dq2;
C(2,1) = -D*sin(q1-q2)*dq1;

% GRAVITY VECTOR
G = zeros(3,1);
G(1) = p.g*p.L1*cos(q1)*(p.m1/2 + p.mM2 + p.m2 + p.m3);
G(2) = p.g*p.L2*cos(q2)*(p.m2/2 + p.m3);
G(3) = 0;

end