%% 3-DOF Planar Arm (RRR) — Forward & Inverse Kinematics
% All three joints rotate about z, so the arm moves in the x-y plane.
% Joint 1 is at the base (origin). Angles are relative (each measured from
% the previous link). End-effector pose = [x; y; phi], where
% phi = q1 + q2 + q3 is the tool orientation.
%
% Requires MATLAB R2016b+ (local functions in scripts). No toolboxes needed.

clear; clc; close all;

%% ---- Arm parameters (edit these) ----
arm.L    = [0.40 0.30 0.05];               % link lengths L1 L2 L3 [m]
arm.qmin = deg2rad([-170 -135 -120]);      % joint lower limits [rad]
arm.qmax = deg2rad([ 170  135  120]);      % joint upper limits [rad]

%% ---- Dynamic parameters (edit these) ----
arm.m  = [2.0 1.5 0.5];                    % link masses [kg]
arm.lc = arm.L / 2;                        % joint-to-COM distance along each link [m]
arm.I  = arm.m .* arm.L.^2 / 12;           % inertia of each link about its COM (z) [kg m^2]
                                           %   (default = uniform slender rod)
% Gripper payload (rigidly held, COM on link 3's axis)
arm.mp = 1.0;                              % payload mass [kg] (0 = no load)
arm.lp = arm.L(3);                         % joint-3-to-payload-COM distance [m] (L3 = at the tip)
arm.Ip = 0.002;                            % payload inertia about its own COM (z) [kg m^2]
                                           %   (include the gripper's mass in arm.m(3) or here)
arm.g  = [0; -9.81];                       % gravity vector in the arm's x-y plane [m/s^2]
                                           %   arm in a vertical plane, y up (rover): [0; -9.81]
                                           %   z vertical (arm swings horizontally):  [0; 0]

%% ---- Joint speed/acceleration limits (edit these; used for worst-case torque) ----
arm.qdmax  = deg2rad([45 45 90]);          % max joint speeds [rad/s]
arm.qddmax = deg2rad([90 90 180]);         % max joint accelerations [rad/s^2]

%% ---- Rover mounting (edit these) ----
% Assumes the arm works in a VERTICAL plane on the rover: arm x = forward,
% arm y = up, joint axes horizontal (sideways across the rover).
% Ground frame: origin on the ground directly below joint 1.
mount.h     = 0.50;                        % height of joint 1 above the ground [m]
mount.clear = 0.02;                        % min clearance of any arm point above ground [m]
mount.box   = [-0.70 0.05 0.10 0.45];      % rover body keep-out [xmin xmax ymin ymax]
                                           %   in the ground frame [m]; [] to ignore the body

%% ---- Forward kinematics example ----
q = deg2rad([30 45 -20]);                  % joint angles [rad]
[pose, P] = fk3R(q, arm);
fprintf('FK:  x = %.4f m,  y = %.4f m,  phi = %.2f deg\n', ...
        pose(1), pose(2), rad2deg(pose(3)));

%% ---- Inverse kinematics example (round trip from the FK pose) ----
[qSol, valid] = ik3R(pose, arm);
labels = {'q2 >= 0', 'q2 <  0'};
if isempty(qSol)
    fprintf('IK:  target out of reach\n');
else
    for k = 1:size(qSol,1)
        err = norm(fk3R(qSol(k,:), arm) - pose);
        fprintf('IK (%s): q = [%7.2f %7.2f %7.2f] deg | in limits: %d | FK error: %.2e\n', ...
                labels{k}, rad2deg(qSol(k,:)), valid(k), err);
    end
end

%% ---- Plot ----
figure; hold on; axis equal; grid on;
th = linspace(0, 2*pi, 200);
plot(sum(arm.L)*cos(th), sum(arm.L)*sin(th), 'k:');     % max reach
colors = {'b', 'r'};
for k = 1:size(qSol,1)
    [~, Pk] = fk3R(qSol(k,:), arm);
    plot(Pk(1,:), Pk(2,:), '-o', 'Color', colors{k}, 'LineWidth', 2, ...
         'MarkerFaceColor', colors{k}, 'DisplayName', ['IK ' labels{k}]);
end
plot(pose(1), pose(2), 'kx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Target');
xlabel('x [m]'); ylabel('y [m]'); title('3R Planar Arm'); legend('show');

%% ---- Dynamics example: joint torques for a rest-to-rest move ----
% Move from straight-out (all zeros) to the IK solution over Tf seconds
% using a cubic profile (zero velocity at both ends).
qA = [0 0 0];
iB = find(valid, 1); if isempty(iB), iB = 1; end
qB = qSol(iB,:);
Tf = 2;                                    % move duration [s]
t  = linspace(0, Tf, 201);
s   = 3*(t/Tf).^2 - 2*(t/Tf).^3;           % position scaling 0 -> 1
sd  = 6*t/Tf^2 - 6*t.^2/Tf^3;              % its time derivative
sdd = 6/Tf^2 - 12*t/Tf^3;                  % its second derivative

tau = zeros(numel(t), 3);
for k = 1:numel(t)
    qk   = qA + s(k)  *(qB - qA);
    qdk  =      sd(k) *(qB - qA);
    qddk =      sdd(k)*(qB - qA);
    tau(k,:) = invDyn3R(qk, qdk, qddk, arm).';
end
fprintf('Peak torques: tau1 = %.3f, tau2 = %.3f, tau3 = %.3f N m\n', max(abs(tau)));

% Worst-case static holding torque: arm straight out horizontally, at rest
tauHold   = invDyn3R([0 0 0], [0 0 0], [0 0 0], arm);
armNoLoad = arm; armNoLoad.mp = 0;
tauHold0  = invDyn3R([0 0 0], [0 0 0], [0 0 0], armNoLoad);
fprintf('Holding torque, arm horizontal: [%.3f %.3f %.3f] N m with %.2f kg payload, [%.3f %.3f %.3f] N m without\n', ...
        tauHold, arm.mp, tauHold0);

figure; plot(t, tau, 'LineWidth', 1.5); grid on;
xlabel('t [s]'); ylabel('Joint torque [N m]');
legend('\tau_1', '\tau_2', '\tau_3'); title('Required joint torques');

% Sanity check: Mdot - 2C must be skew-symmetric for a correct C
kMid = round(numel(t)/2);
qk = qA + s(kMid)*(qB - qA);  qdk = sd(kMid)*(qB - qA);
[~, C, ~, dMdq] = dyn3R(qk, qdk, arm);
Mdot = dMdq(:,:,1)*qdk(1) + dMdq(:,:,2)*qdk(2) + dMdq(:,:,3)*qdk(3);
N = Mdot - 2*C;
fprintf('Skew-symmetry check ||N + N''|| = %.2e (should be ~0)\n', norm(N + N.'));

%% ---- Worst-case joint torques ----
% Searches every pose within the joint limits and clear of the ground and
% rover body (coarse grid, then refined) for the largest torque each joint
% can see:
%   Static  = holding still (gravity only)
%   Dynamic = gravity + joints moving at up to arm.qdmax and accelerating
%             at up to arm.qddmax, in the worst combination of directions.
%             This is a conservative upper bound: a real move rarely hits
%             full speed and full acceleration on every joint at once.
% Set arm.mp to the heaviest load the arm will carry.
wc = worstCaseTorque(arm, mount, 12);      % 12 grid points per joint for the coarse search
fprintf('\nWorst-case joint torques (payload %.2f kg):\n', arm.mp);
fprintf('  Joint |  Static [N m] | Dynamic [N m] | Pose for dynamic worst case [deg]\n');
for i = 1:numel(arm.L)
    fprintf('    %d   | %12.3f  | %12.3f  | [%s]\n', i, wc.static(i), wc.dyn(i), ...
            sprintf('%7.1f', rad2deg(wc.qDyn(i,:))));
end

figure;
for i = 1:numel(arm.L)
    subplot(2, ceil(numel(arm.L)/2), i); hold on; axis equal; grid on;
    plotScene(mount);
    drawArm(wc.qStatic(i,:), arm, mount, [0.5 0.5 0.5], 1.5, 'on', ...
            sprintf('Static %.1f N m', wc.static(i)));
    drawArm(wc.qDyn(i,:), arm, mount, [0.85 0.2 0.1], 2, 'on', ...
            sprintf('Dynamic %.1f N m', wc.dyn(i)));
    title(sprintf('Joint %d worst case', i));
    xlabel('x [m]'); ylabel('Height [m]'); legend('show', 'Location', 'best');
end

%% ---- Reach envelope on the rover ----
env = reachEnvelope3R(arm, mount, 50);     % 50 samples per joint
figure; hold on; axis equal; grid on;
plot(env.tip(:,1), env.tip(:,2), '.', 'Color', [0.75 0.85 1], 'MarkerSize', 4, ...
     'HandleVisibility', 'off');
plotEnvelope(env, mount, 'b', sprintf('Envelope, h = %.2f m', mount.h));
plotScene(mount);
xlabel('Forward x [m]'); ylabel('Height above ground [m]');
title('Arm reach envelope on rover'); legend('show', 'Location', 'best');

% Check whether a specific point is reachable (any tool angle)
target = [0.55 0.05];                      % [forward x, height] in ground frame [m]
                                           %   (height must be >= mount.clear)
[canReach, qT] = canReach3R(target, arm, mount);
if canReach
    fprintf('Target (%.2f, %.2f) m reachable with q = [%.1f %.1f %.1f] deg\n', ...
            target, rad2deg(qT));
else
    fprintf('Target (%.2f, %.2f) m NOT reachable\n', target);
end

%% ---- TEST: move to a specific point with a specific gripper angle ----
% Edit these lines to test a move. Positions are in the ground frame.
testTarget  = [0.55 0.05];                 % goal tip [forward x, height above ground] [m]
testPhi     = deg2rad(-90);                % goal gripper angle from horizontal (-90 = pointing down)

% Start: as a tip position + gripper angle, or as joint angles
testStartMode  = 'position';               % 'position' or 'angles'
testStartPos   = [0.30 0.60];              % start tip [forward x, height above ground] [m]
testStartPhi   = deg2rad(-90);             % start gripper angle [rad]
testStartElbow = 'up';                     % 'up' or 'down': preferred start elbow if both work
qStartAngles   = deg2rad([60 -120 -30]);   % start joint angles, used if testStartMode = 'angles'

testTf      = 3.5;                         % move duration [s]
testAnimate = true;                        % play an animation of the move

fprintf('\n--- Test move to (%.3f, %.3f) m at phi = %.1f deg ---\n', ...
        testTarget, rad2deg(testPhi));
% Starting pose
if strcmpi(testStartMode, 'position')
    [qStart, msg] = solveTarget3R(testStartPos, testStartPhi, arm, mount, testStartElbow);
    if isempty(qStart)
        error('Start position (%.3f, %.3f) m at phi = %.1f deg: %s', ...
              testStartPos(1), testStartPos(2), rad2deg(testStartPhi), msg);
    end
else
    qStart = qStartAngles;
    if ~clearOfObstacles(qStart, arm, mount) || any(qStart < arm.qmin) || any(qStart > arm.qmax)
        warning('Starting pose violates joint limits or collides with ground/rover.');
    end
end
fprintf('Start joint angles: [%.1f %.1f %.1f] deg\n', rad2deg(qStart));

% Goal pose: of the usable IK solutions, the one needing least joint travel
[qGoal, msg] = solveTarget3R(testTarget, testPhi, arm, mount, qStart);
if isempty(qGoal)
    fprintf('FAIL: %s.\n', msg);
else
    fprintf('Goal joint angles:  [%.1f %.1f %.1f] deg\n', rad2deg(qGoal));
end

if ~isempty(qGoal)
    % Smooth rest-to-rest joint trajectory (cubic)
    tT   = linspace(0, testTf, 201);
    sT   = 3*(tT/testTf).^2 - 2*(tT/testTf).^3;
    sdT  = 6*tT/testTf^2 - 6*tT.^2/testTf^3;
    sddT = 6/testTf^2 - 12*tT/testTf^3;
    dq   = qGoal - qStart;
    qPath = qStart + sT.' * dq;            % 201 x 3

    % Check the move against the joint speed/acceleration limits
    % (cubic profile: peak speed = 1.5*dq/Tf, peak accel = 6*dq/Tf^2)
    TfMin = max([1.5*abs(dq)./arm.qdmax, sqrt(6*abs(dq)./arm.qddmax)]);
    if testTf < TfMin
        fprintf('WARNING: move is too fast for the joint speed/accel limits; needs testTf >= %.2f s.\n', TfMin);
    end

    % Collision check along the whole path, not just the endpoints
    pathOK = clearOfObstacles(qPath, arm, mount);
    if all(pathOK)
        fprintf('Path is collision-free.\n');
    else
        fprintf('WARNING: path collides at t = %.2f s (try another qStart or add waypoints).\n', ...
                tT(find(~pathOK, 1)));
    end

    % Joint torques along the move (includes payload)
    tauT = zeros(numel(tT), 3);
    for k = 1:numel(tT)
        tauT(k,:) = invDyn3R(qPath(k,:), sdT(k)*dq, sddT(k)*dq, arm).';
    end
    tauEnd = invDyn3R(qGoal, [0 0 0], [0 0 0], arm);
    fprintf('Peak torques during move: [%.3f %.3f %.3f] N m\n', max(abs(tauT)));
    fprintf('Holding torque at target: [%.3f %.3f %.3f] N m\n', tauEnd);

    % Plot: envelope, ghosted motion, start and final pose, target + approach
    figure; hold on; axis equal; grid on;
    plotScene(mount);
    plotEnvelope(env, mount, [0.55 0.7 1], 'Reach envelope');
    for k = round(linspace(1, numel(tT), 9))
        drawArm(qPath(k,:), arm, mount, [0.75 0.75 0.75], 1, 'off', '');
    end
    drawArm(qStart, arm, mount, [0.3 0.3 0.3], 1.5, 'on', 'Start');
    drawArm(qGoal,  arm, mount, [0.85 0.2 0.1], 2.5, 'on', 'Final');
    plot(testTarget(1), testTarget(2), 'kx', 'MarkerSize', 12, 'LineWidth', 2, ...
         'DisplayName', 'Target');
    quiver(testTarget(1) - 0.1*cos(testPhi), testTarget(2) - 0.1*sin(testPhi), ...
           0.1*cos(testPhi), 0.1*sin(testPhi), 0, 'k', 'LineWidth', 1.5, ...
           'MaxHeadSize', 1, 'DisplayName', 'Approach direction');
    xlabel('Forward x [m]'); ylabel('Height above ground [m]');
    title('Test move'); legend('show', 'Location', 'best');

    figure; plot(tT, tauT, 'LineWidth', 1.5); grid on;
    xlabel('t [s]'); ylabel('Joint torque [N m]');
    legend('\tau_1', '\tau_2', '\tau_3'); title('Test move: joint torques');

    % Animation
    if testAnimate
        figure; hold on; axis equal; grid on;
        plotScene(mount);
        plot(testTarget(1), testTarget(2), 'kx', 'MarkerSize', 12, 'LineWidth', 2);
        R = sum(arm.L);
        axis([-R-0.1, R+0.1, -0.05, mount.h + R + 0.1]);
        hA = drawArm(qStart, arm, mount, [0.85 0.2 0.1], 2.5, 'off', '');
        title('Test move (animation)');
        for k = 1:2:numel(tT)
            [~, X, Y] = clearOfObstacles(qPath(k,:), arm, mount);
            set(hA, 'XData', X, 'YData', Y);
            drawnow;
            pause(2*testTf/numel(tT));
        end
    end
end

%% ======================= Functions =======================

function [pose, P] = fk3R(q, arm)
% FK3R  Forward kinematics of a planar 3R arm.
%   q    : 1x3 joint angles [rad]
%   pose : [x; y; phi] of the end effector
%   P    : 2x4 joint positions [base, joint2, joint3, tip] for plotting
    q = q(:).';
    if any(q < arm.qmin) || any(q > arm.qmax)
        warning('fk3R:limits', 'Joint angles outside range of motion.');
    end
    L = arm.L(:).';
    a = cumsum(q);                         % absolute link angles
    x = [0, cumsum(L .* cos(a))];
    y = [0, cumsum(L .* sin(a))];
    phi = atan2(sin(a(3)), cos(a(3)));     % wrap to [-pi, pi]
    pose = [x(end); y(end); phi];
    P = [x; y];
end

function [qSol, valid] = ik3R(pose, arm)
% IK3R  Inverse kinematics of a planar 3R arm.
%   pose  : [x; y; phi] desired end-effector position and orientation
%   qSol  : 2x3 joint solutions [rad], row 1 has q2 >= 0, row 2 has q2 < 0
%           (empty if the target is out of reach)
%   valid : 2x1 logical, true if that solution is within joint limits
    L1 = arm.L(1); L2 = arm.L(2); L3 = arm.L(3);
    x = pose(1); y = pose(2); phi = pose(3);

    % Wrist (joint 3) position: back off from the tip along the tool axis
    xw = x - L3*cos(phi);
    yw = y - L3*sin(phi);

    % Law of cosines for joint 2
    c2 = (xw^2 + yw^2 - L1^2 - L2^2) / (2*L1*L2);
    tol = 1e-9;
    if abs(c2) > 1 + tol
        qSol = []; valid = false(0,1);
        return
    end
    c2 = max(min(c2, 1), -1);              % clamp for numerical safety

    qSol  = zeros(2,3);
    valid = false(2,1);
    sgn = [1, -1];                         % two elbow configurations
    for k = 1:2
        s2 = sgn(k) * sqrt(1 - c2^2);
        q2 = atan2(s2, c2);
        q1 = atan2(yw, xw) - atan2(L2*s2, L1 + L2*c2);
        q3 = phi - q1 - q2;
        [qSol(k,:), valid(k)] = fitToLimits([q1 q2 q3], arm.qmin, arm.qmax);
    end
end

function [q, ok] = fitToLimits(q, qmin, qmax)
% Wrap each angle and pick the 2*pi-equivalent that lies within its limits.
    tol = 1e-9;
    ok = true;
    for i = 1:numel(q)
        qi = atan2(sin(q(i)), cos(q(i)));
        cands = qi + 2*pi*[0 -1 1];
        idx = find(cands >= qmin(i) - tol & cands <= qmax(i) + tol, 1);
        if isempty(idx)
            q(i) = qi; ok = false;
        else
            q(i) = cands(idx);
        end
    end
end

function [M, C, G, dMdq, Cs] = dyn3R(q, qd, arm)
% DYN3R  Equation-of-motion terms for the planar 3R arm:
%           M(q)*qdd + C(q,qd)*qd + G(q) = tau
%   q, qd : joint angles [rad] and rates [rad/s]
%   M     : 3x3 mass matrix
%   C     : 3x3 Coriolis/centrifugal matrix (Christoffel form)
%   G     : 3x1 gravity torques
%   dMdq  : 3x3x3, dMdq(:,:,k) = dM/dq_k
%   Cs    : 3x3x3 Christoffel symbols, so that the velocity torque on
%           joint i is sum over j,k of Cs(i,j,k)*qd(j)*qd(k)
    q = q(:); qd = qd(:);
    a = cumsum(q);                         % absolute link angles
    T = tril(ones(3));                     % a = T*q

    % Link 3 properties including the gripper payload
    [m, lc, I] = withPayload(arm);
    m = m(:);

    % r(i,j) = lever arm from joint j toward the COM of link i
    r = zeros(3);
    for i = 1:3
        for j = 1:i-1
            r(i,j) = arm.L(j);
        end
        r(i,i) = lc(i);
    end

    % Kinetic energy = 1/2 adot' * Ma * adot, with Ma(j,k) = D(j,k)*cos(a_j - a_k)
    D  = r.' * diag(m) * r + diag(I);
    dA = a - a.';
    Ma = D .* cos(dA);
    M  = T.' * Ma * T;                     % convert to relative joint angles

    % Partial derivatives of M, by the chain rule through the absolute angles
    dMa = zeros(3,3,3);
    for l = 1:3
        E = zeros(3); E(l,:) = 1; E(:,l) = E(:,l) - 1;   % delta_jl - delta_kl
        dMa(:,:,l) = -D .* sin(dA) .* E;
    end
    dMdq = zeros(3,3,3);
    for k = 1:3
        S = zeros(3);
        for l = 1:3
            S = S + T(l,k) * dMa(:,:,l);
        end
        dMdq(:,:,k) = T.' * S * T;
    end

    % Coriolis/centrifugal matrix from Christoffel symbols
    Cs = zeros(3,3,3);
    C = zeros(3);
    for i = 1:3
        for j = 1:3
            for k = 1:3
                Cs(i,j,k) = 0.5*(dMdq(i,j,k) + dMdq(i,k,j) - dMdq(j,k,i));
                C(i,j) = C(i,j) + Cs(i,j,k) * qd(k);
            end
        end
    end

    % Gravity: G = dV/dq, where V = -sum_i m_i * g . p_ci
    gx = arm.g(1); gy = arm.g(2);
    dVda = -(r.' * m) .* (-gx*sin(a) + gy*cos(a));
    G = T.' * dVda;
end

function [m, lc, I] = withPayload(arm)
% WITHPAYLOAD  Combine link 3 and the payload into one rigid body:
%   combined mass, COM distance from joint 3, and inertia about the new COM
%   (parallel-axis theorem). Links 1 and 2 are unchanged.
    m = arm.m; lc = arm.lc; I = arm.I;
    if isfield(arm, 'mp') && arm.mp > 0
        m3 = m(3) + arm.mp;
        c3 = (m(3)*lc(3) + arm.mp*arm.lp) / m3;
        I(3) = I(3) + m(3)*(lc(3) - c3)^2 + arm.Ip + arm.mp*(arm.lp - c3)^2;
        m(3) = m3;
        lc(3) = c3;
    end
end

function tau = invDyn3R(q, qd, qdd, arm)
% INVDYN3R  Joint torques needed to produce motion (q, qd, qdd).
    [M, C, G] = dyn3R(q, qd, arm);
    tau = M*qdd(:) + C*qd(:) + G;
end

function qdd = fwdDyn3R(q, qd, tau, arm)
% FWDDYN3R  Joint accelerations produced by torques tau (for simulation,
%   e.g. inside an ode45 right-hand side).
    [M, C, G] = dyn3R(q, qd, arm);
    qdd = M \ (tau(:) - C*qd(:) - G);
end

function wc = worstCaseTorque(arm, mount, N)
% WORSTCASETORQUE  Largest torque each joint can see, over every pose within
%   the joint limits and clear of the ground and rover body.
%   N : grid points per joint for the coarse search (default 12)
%   wc.static(i), wc.qStatic(i,:) : worst holding torque on joint i and its pose
%   wc.dyn(i),    wc.qDyn(i,:)    : worst moving torque on joint i and its pose
%   wc.vDyn(i,:), wc.aDyn(i,:)    : joint speeds and accelerations causing it
% Method: at each pose, torque_i = M_i*qdd + (velocity terms)_i + G_i.
%   It is linear in qdd, so its worst value is |velocity + gravity| +
%   sum_j |M_ij|*qddmax_j. Velocity terms are checked at every combination
%   of {-qdmax, 0, +qdmax}. The best coarse-grid poses are then refined
%   with a pattern search that stays within the valid poses.
    if nargin < 3, N = 12; end
    n = numel(arm.L);

    % Coarse grid of valid poses (always includes 0 = links in line)
    g = cell(1, n);
    for j = 1:n
        g{j} = linspace(arm.qmin(j), arm.qmax(j), N);
        if arm.qmin(j) < 0 && arm.qmax(j) > 0, g{j} = unique([g{j} 0]); end
    end
    Gd = cell(1, n); [Gd{:}] = ndgrid(g{:});
    Q = zeros(numel(Gd{1}), n);
    for j = 1:n, Q(:,j) = Gd{j}(:); end
    Q = Q(clearOfObstacles(Q, arm, mount), :);
    if isempty(Q)
        error('worstCaseTorque: no valid poses found; check limits, mount and rover box.');
    end

    % Joint speed combinations to test
    Vd = cell(1, n); [Vd{:}] = ndgrid([-1 0 1]);
    V = zeros(3^n, n);
    for j = 1:n, V(:,j) = Vd{j}(:) * arm.qdmax(j); end

    K = size(Q, 1);
    tS = zeros(K, n); tD = zeros(K, n);
    for k = 1:K
        [tS(k,:), tD(k,:)] = poseWorstTorque(Q(k,:), arm, V);
    end

    % Refine the best few coarse poses for each joint
    step0 = max(arm.qmax - arm.qmin) / (N - 1);  % start at the grid spacing
    wc.static = zeros(1,n); wc.dyn = zeros(1,n);
    wc.qStatic = zeros(n); wc.qDyn = zeros(n);
    wc.vDyn = zeros(n); wc.aDyn = zeros(n);
    for i = 1:n
        for mode = 1:2                     % 1 = static, 2 = dynamic
            if mode == 1, t = tS(:,i); else, t = tD(:,i); end
            [~, order] = sort(t, 'descend');
            bestVal = -inf; bestQ = [];
            for c = order(1:min(5, K)).'
                f = @(q) validTorque(q, i, mode, arm, mount, V);
                [qr, vr] = patternSearchMax(f, Q(c,:), t(c), step0);
                if vr > bestVal, bestVal = vr; bestQ = qr; end
            end
            if mode == 1
                wc.static(i) = bestVal; wc.qStatic(i,:) = bestQ;
            else
                wc.dyn(i) = bestVal; wc.qDyn(i,:) = bestQ;
                [~, ~, wc.vDyn(i,:), wc.aDyn(i,:)] = poseWorstTorque(bestQ, arm, V, i);
            end
        end
    end
end

function [x, fx] = patternSearchMax(f, x, fx, step)
% Maximize f by stepping each joint +/- step, halving the step when no
% move helps. Invalid poses score 0, so the search never leaves the
% valid region. Stops when the step is below 0.05 deg.
    n = numel(x);
    while step > deg2rad(0.05)
        improved = false;
        for j = 1:n
            for s = [1 -1]
                xt = x; xt(j) = xt(j) + s*step;
                ft = f(xt);
                if ft > fx
                    x = xt; fx = ft; improved = true;
                end
            end
        end
        if ~improved, step = step / 2; end
    end
end

function t = validTorque(q, i, mode, arm, mount, V)
% Worst torque on joint i at pose q, or 0 if q is outside the joint limits
% or collides (so the optimizer stays within the valid poses).
    if any(q < arm.qmin) || any(q > arm.qmax) || ~clearOfObstacles(q, arm, mount)
        t = 0; return
    end
    [tS, tD] = poseWorstTorque(q, arm, V);
    if mode == 1, t = tS(i); else, t = tD(i); end
end

function [tStat, tDyn, vW, aW] = poseWorstTorque(q, arm, V, iOut)
% Worst static and dynamic torque on each joint at pose q.
%   V : P x n joint speed combinations to test
%   vW, aW : speeds and accelerations giving joint iOut's worst dynamic torque
    n = numel(q);
    [M, ~, Gv, ~, Cs] = dyn3R(q, zeros(1,n), arm);
    tStat = abs(Gv).';
    tDyn = zeros(1, n);
    vW = zeros(1, n); aW = zeros(1, n);
    for i = 1:n
        Ai = reshape(Cs(i,:,:), n, n);
        h = Gv(i) + sum((V*Ai) .* V, 2);   % gravity + velocity torque, per speed combination
        [hMax, p] = max(abs(h));
        tDyn(i) = hMax + abs(M(i,:)) * arm.qddmax(:);
        if nargin > 3 && i == iOut
            vW = V(p,:);
            sh = sign(h(p)); if sh == 0, sh = 1; end
            aW = sh * sign(M(i,:)) .* arm.qddmax;
        end
    end
end

function [ok, X, Y] = clearOfObstacles(q, arm, mount)
% CLEAROFOBSTACLES  For each row of q (K x 3), true if every link stays at
%   least mount.clear above the ground and outside the rover keep-out box.
%   X, Y : K x 4 positions [base, joint2, joint3, tip] in the ground frame.
%   Links are straight, so the ground check only needs the joint points;
%   the box check samples 10 points along each link.
    n = size(q, 1);
    a = cumsum(q, 2);
    X = [zeros(n,1), cumsum(arm.L .* cos(a), 2)];
    Y = mount.h + [zeros(n,1), cumsum(arm.L .* sin(a), 2)];
    ok = all(Y(:,2:end) >= mount.clear, 2);
    if ~isempty(mount.box)
        b = mount.box;
        if 0 > b(1) && 0 < b(2) && mount.h > b(3) && mount.h < b(4)
            error('Joint 1 is inside the rover keep-out box; raise mount.h or edit mount.box.');
        end
        for i = 1:3
            for s = 0.1:0.1:1
                xs = X(:,i) + s*(X(:,i+1) - X(:,i));
                ys = Y(:,i) + s*(Y(:,i+1) - Y(:,i));
                ok = ok & ~(xs > b(1) & xs < b(2) & ys > b(3) & ys < b(4));
            end
        end
    end
end

function env = reachEnvelope3R(arm, mount, N)
% REACHENVELOPE3R  Sample the joint space (N values per joint, within the
%   limits) and keep the tip positions of collision-free poses.
%   env.tip : K x 2 reachable tip points [forward x, height above ground]
%   env.q   : K x 3 joint angles reaching them
%   Also prints the key reach numbers for this mount height.
    if nargin < 3, N = 50; end
    [Q1, Q2, Q3] = ndgrid(linspace(arm.qmin(1), arm.qmax(1), N), ...
                          linspace(arm.qmin(2), arm.qmax(2), N), ...
                          linspace(arm.qmin(3), arm.qmax(3), N));
    q = [Q1(:) Q2(:) Q3(:)];
    [ok, X, Y] = clearOfObstacles(q, arm, mount);
    env.tip = [X(ok,end) Y(ok,end)];
    env.q   = q(ok,:);

    gnd = env.tip(:,2) <= mount.clear + 0.02;   % tip within 2 cm of its lowest allowed height
    if any(gnd)
        gx = env.tip(gnd,1);
        gStr = sprintf('%.3f to %.3f m', min(gx), max(gx));
    else
        gStr = 'cannot reach ground';
    end
    fprintf(['Mount h = %.2f m: max forward reach %.3f m, max tip height %.3f m, ' ...
             'ground reach x = %s\n'], mount.h, max(env.tip(:,1)), max(env.tip(:,2)), gStr);
end

function [ok, q] = canReach3R(target, arm, mount, nPhi)
% CANREACH3R  Can the tip reach target = [forward x, height above ground]
%   with any tool angle? Sweeps phi, runs ik3R, and checks the joint
%   limits, ground clearance and rover keep-out box.
    if nargin < 4, nPhi = 360; end
    for phi = linspace(-pi, pi, nPhi)
        pose = [target(1); target(2) - mount.h; phi];   % ground frame -> arm frame
        [qs, valid] = ik3R(pose, arm);
        for k = 1:numel(valid)
            if valid(k) && clearOfObstacles(qs(k,:), arm, mount)
                ok = true; q = qs(k,:);
                return
            end
        end
    end
    ok = false; q = [];
end

function plotEnvelope(env, mount, col, name)
% Outline of the reachable region plus the mount point.
    kb = boundary(env.tip(:,1), env.tip(:,2), 0.9);
    plot(env.tip(kb,1), env.tip(kb,2), '-', 'Color', col, 'LineWidth', 2, ...
         'DisplayName', name);
    plot(0, mount.h, 's', 'Color', col, 'MarkerFaceColor', col, ...
         'HandleVisibility', 'off');
end

function [q, msg] = solveTarget3R(target, phi, arm, mount, pref)
% SOLVETARGET3R  IK for a ground-frame tip target [x, height] with gripper
%   angle phi. Returns a usable solution (within limits, collision-free):
%     pref = 1x3 joint angles -> solution needing least joint travel from it
%     pref = 'up' or 'down'   -> preferred elbow, falling back to the other
%   q is [] if no solution is usable, and msg says why.
    q = []; msg = '';
    pose = [target(1); target(2) - mount.h; phi];   % ground frame -> arm frame
    [qs, v] = ik3R(pose, arm);
    if isempty(qs)
        msg = 'out of reach (wrist point too far from or too close to joint 1)';
        return
    end
    if ~any(v)
        msg = 'reachable, but every solution violates joint limits at this phi';
        return
    end
    ok = v & clearOfObstacles(qs, arm, mount);
    if ~any(ok)
        msg = 'solutions within limits collide with the ground or rover body';
        return
    end
    idx = find(ok);
    if isnumeric(pref)
        [~, j] = min(sum(abs(qs(idx,:) - pref(:).'), 2));
        q = qs(idx(j),:);
    else
        % Row 2 of ik3R has q2 < 0 (elbow up when reaching forward), row 1 has q2 >= 0
        want = 1 + strcmpi(pref, 'up');
        if ok(want)
            q = qs(want,:);
        else
            q = qs(idx(1),:);
        end
    end
end

function h = drawArm(q, arm, mount, col, lw, legendVis, name)
% Draw the arm at joint angles q in the ground frame; returns the line handle.
    [~, X, Y] = clearOfObstacles(q, arm, mount);
    h = plot(X, Y, '-o', 'Color', col, 'LineWidth', lw, 'MarkerSize', 4, ...
             'MarkerFaceColor', col, 'HandleVisibility', legendVis, 'DisplayName', name);
end

function plotScene(mount)
% Ground line and rover keep-out box.
    yline(0, 'k-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    if ~isempty(mount.box)
        b = mount.box;
        fill(b([1 2 2 1]), b([3 3 4 4]), [0.6 0.6 0.6], 'FaceAlpha', 0.4, ...
             'EdgeColor', 'k', 'DisplayName', 'Rover body');
    end
end
