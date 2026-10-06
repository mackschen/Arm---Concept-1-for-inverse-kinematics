%% 4-DOF Planar Arm (RRRR) — Kinematics, Dynamics, Reach Envelope & Test Moves
% All four joint axes are parallel, so the arm moves in one plane.
%   Joint 1 = base, Joint 2 = elbow, Joint 3 = added joint, Joint 4 = wrist
%   Link 4 = gripper (short)
% Angles: q1 is measured from horizontal; q2, q3, q4 are measured relative
% to the previous link (0 = in line with it). Positive = counterclockwise.
% Gripper angle from horizontal: phi = q1 + q2 + q3 + q4.
%
% REDUNDANCY: the arm has 4 joints, but a target (x, height, gripper angle)
% is only 3 numbers, so infinitely many arm poses reach the same target.
% The IK handles this with one extra parameter, psi = the angle of link 3
% from horizontal. It tries many values of psi and picks the best pose:
% the least joint travel from a reference pose, or the most margin from
% the joint limits.
%
% Requires MATLAB R2016b+ (local functions in scripts). No toolboxes needed.

clear; clc; close all;

%% ---- Arm parameters (edit these) ----
arm.L    = [0.40 0.30 0.20 0.05];          % link lengths L1 L2 L3 L4 [m] (L4 = gripper)
arm.qmin = deg2rad([0 -90 -90 -90]); % joint lower limits [rad]
arm.qmax = deg2rad([ 180  90  90  90]); % joint upper limits [rad]

%% ---- Dynamic parameters (edit these) ----
arm.m  = [2.0 1.5 1.0 0.3];                % link masses [kg] (link 4 = gripper)
arm.lc = arm.L / 2;                        % joint-to-COM distance along each link [m]
arm.I  = arm.m .* arm.L.^2 / 12;           % inertia of each link about its COM [kg m^2]
                                           %   (default = uniform slender rod)
% Gripper payload (rigidly held, COM on the gripper's axis)
arm.mp = 1.0;                              % payload mass [kg] (0 = no load)
arm.lp = arm.L(4);                         % wrist-joint (joint 4)-to-payload-COM distance [m]
                                           %   (L4 = at the gripper tip)
arm.Ip = 0.002;                            % payload inertia about its own COM [kg m^2]
arm.g  = [0; -9.81];                       % gravity in the arm's plane [m/s^2]
                                           %   arm in a vertical plane, y up (rover): [0; -9.81]

%% ---- Rover mounting (edit these) ----
% Arm works in a VERTICAL plane: x = forward, y = up.
% Ground frame: origin on the ground directly below joint 1.
mount.h     = 0.50;                        % height of joint 1 above the ground [m]
mount.clear = 0.02;                        % min clearance of any arm point above ground [m]
mount.box   = [-0.70 0.05 0.10 0.45];      % rover body keep-out [xmin xmax ymin ymax]
                                           %   in the ground frame [m]; [] to ignore

%% ---- IK settings ----
arm.nPsi = 360;                            % number of link-3 angles the IK tries
                                           %   (higher = finer search, slower)

%% ---- Forward kinematics example ----
q = deg2rad([30 45 -30 -20]);              % joint angles [rad]
pose = fkArm(q, arm);
fprintf('FK:  x = %.4f m,  y = %.4f m,  phi = %.2f deg\n', ...
        pose(1), pose(2), rad2deg(pose(3)));

%% ---- Inverse kinematics example (round trip from the FK pose) ----
% Arm frame here (origin at joint 1, no ground or rover checks).
[qAll, inLim] = ikArm(pose, linspace(-pi, pi, arm.nPsi), arm);
qAll = qAll(inLim, :);
fprintf('IK:  %d candidate poses within joint limits reach this target\n', size(qAll,1));
if ~isempty(qAll)
    qNear = ikBest(pose, arm, q, []);              % closest to the FK input
    qSafe = ikBest(pose, arm, 'margin', []);       % farthest from joint limits
    fprintf('IK (closest to input):  q = [%7.2f %7.2f %7.2f %7.2f] deg | FK error %.1e\n', ...
            rad2deg(qNear), norm(fkArm(qNear, arm) - pose));
    fprintf('IK (most limit margin): q = [%7.2f %7.2f %7.2f %7.2f] deg | FK error %.1e\n', ...
            rad2deg(qSafe), norm(fkArm(qSafe, arm) - pose));

    figure; hold on; axis equal; grid on;
    th = linspace(0, 2*pi, 200);
    plot(sum(arm.L)*cos(th), sum(arm.L)*sin(th), 'k:', 'DisplayName', 'Max reach');
    for k = round(linspace(1, size(qAll,1), 12))
        [~, Pk] = fkArm(qAll(k,:), arm);
        plot(Pk(1,:), Pk(2,:), '-', 'Color', [0.75 0.75 0.75], 'HandleVisibility', 'off');
    end
    [~, Pn] = fkArm(qNear, arm);
    [~, Ps] = fkArm(qSafe, arm);
    plot(Pn(1,:), Pn(2,:), '-o', 'Color', 'b', 'LineWidth', 2, 'MarkerFaceColor', 'b', ...
         'DisplayName', 'Closest to input');
    plot(Ps(1,:), Ps(2,:), '-o', 'Color', 'r', 'LineWidth', 2, 'MarkerFaceColor', 'r', ...
         'DisplayName', 'Most limit margin');
    plot(pose(1), pose(2), 'kx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Target');
    xlabel('x [m]'); ylabel('y [m]');
    title('4R arm: some of the poses reaching one target (grey)'); legend('show');
end

%% ---- Dynamics checks ----
% Worst-case static holding torque: arm straight out horizontally, at rest
z = zeros(1,4);
tauHold   = invDynArm(z, z, z, arm);
armNoLoad = arm; armNoLoad.mp = 0;
tauHold0  = invDynArm(z, z, z, armNoLoad);
fprintf('Holding torque, arm horizontal, %.2f kg payload: [%.3f %.3f %.3f %.3f] N m\n', ...
        arm.mp, tauHold);
fprintf('Holding torque, arm horizontal, no payload:      [%.3f %.3f %.3f %.3f] N m\n', ...
        tauHold0);

% Sanity check: Mdot - 2C must be skew-symmetric for a correct C
qk = deg2rad([20 -40 30 10]);  qdk = [0.5 -0.3 0.8 0.2];
[~, C, ~, dMdq] = dynArm(qk, qdk, arm);
Mdot = zeros(4);
for k = 1:4, Mdot = Mdot + dMdq(:,:,k)*qdk(k); end
N = Mdot - 2*C;
fprintf('Skew-symmetry check ||N + N''|| = %.2e (should be ~0)\n', norm(N + N.'));

%% ---- Reach envelope on the rover ----
env = reachEnvelopeArm(arm, mount, 30);    % 30 samples per joint (30^4 poses)
figure; hold on; axis equal; grid on;
plot(env.tip(:,1), env.tip(:,2), '.', 'Color', [0.75 0.85 1], 'MarkerSize', 4, ...
     'HandleVisibility', 'off');
plotEnvelope(env, mount, 'b', sprintf('Envelope, h = %.2f m', mount.h));
plotScene(mount);
xlabel('Forward x [m]'); ylabel('Height above ground [m]');
title('Arm reach envelope on rover'); legend('show', 'Location', 'best');

% Check whether a specific point is reachable (any gripper angle)
target = [0.55 0.05];                      % [forward x, height] in ground frame [m]
                                           %   (height must be >= mount.clear)
[canReach, qT] = canReachArm(target, arm, mount);
if canReach
    fprintf('Target (%.2f, %.2f) m reachable, e.g. q = [%.1f %.1f %.1f %.1f] deg, phi = %.1f deg\n', ...
            target(1), target(2), rad2deg(qT), rad2deg(sum(qT)));
else
    fprintf('Target (%.2f, %.2f) m NOT reachable\n', target(1), target(2));
end

%% ---- TEST: move to a specific point with a specific gripper angle ----
% Edit these lines to test a move. Positions are in the ground frame.
testTarget  = [0.55 0.05];                 % goal tip [forward x, height above ground] [m]
testPhi     = deg2rad(-90);                % goal gripper angle from horizontal (-90 = pointing down)

% Start: as a tip position + gripper angle, or as joint angles
testStartMode  = 'position';               % 'position' or 'angles'
testStartPos   = [0.30 0.60];              % start tip [forward x, height above ground] [m]
testStartPhi   = deg2rad(-90);             % start gripper angle [rad]
testStartPref  = 'margin';                 % how to pick the start pose among all that work:
                                           %   'margin' = farthest from joint limits
                                           %   'up' / 'down' = elbow (joint 2) up or down
qStartAngles   = deg2rad([60 -100 -40 -10]); % start joint angles, used if testStartMode = 'angles'

testTf      = 2.0;                         % move duration [s]
testAnimate = true;                        % play an animation of the move

fprintf('\n--- Test move to (%.3f, %.3f) m at phi = %.1f deg ---\n', ...
        testTarget(1), testTarget(2), rad2deg(testPhi));
% Starting pose
if strcmpi(testStartMode, 'position')
    [qStart, msg] = solveTargetArm(testStartPos, testStartPhi, arm, mount, testStartPref);
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
fprintf('Start joint angles: [%.1f %.1f %.1f %.1f] deg\n', rad2deg(qStart));

% Goal pose: of all usable poses, the one needing least joint travel from the start
[qGoal, msg] = solveTargetArm(testTarget, testPhi, arm, mount, qStart);
if isempty(qGoal)
    fprintf('FAIL: %s.\n', msg);
else
    fprintf('Goal joint angles:  [%.1f %.1f %.1f %.1f] deg\n', rad2deg(qGoal));
end

if ~isempty(qGoal)
    % Smooth rest-to-rest joint trajectory (cubic)
    tT   = linspace(0, testTf, 201);
    sT   = 3*(tT/testTf).^2 - 2*(tT/testTf).^3;
    sdT  = 6*tT/testTf^2 - 6*tT.^2/testTf^3;
    sddT = 6/testTf^2 - 12*tT/testTf^3;
    dq   = qGoal - qStart;
    qPath = qStart + sT.' * dq;            % 201 x 4

    % Collision check along the whole path, not just the endpoints
    pathOK = clearOfObstacles(qPath, arm, mount);
    if all(pathOK)
        fprintf('Path is collision-free.\n');
    else
        fprintf('WARNING: path collides at t = %.2f s (try another start or add waypoints).\n', ...
                tT(find(~pathOK, 1)));
    end

    % Joint torques along the move (includes payload)
    tauT = zeros(numel(tT), 4);
    for k = 1:numel(tT)
        tauT(k,:) = invDynArm(qPath(k,:), sdT(k)*dq, sddT(k)*dq, arm).';
    end
    tauEnd = invDynArm(qGoal, z, z, arm);
    fprintf('Peak torques during move: [%.3f %.3f %.3f %.3f] N m\n', max(abs(tauT)));
    fprintf('Holding torque at target: [%.3f %.3f %.3f %.3f] N m\n', tauEnd);

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
    legend('\tau_1', '\tau_2', '\tau_3', '\tau_4'); title('Test move: joint torques');

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

function [pose, P] = fkArm(q, arm)
% FKARM  Forward kinematics of the planar 4R arm.
%   q    : 1x4 joint angles [rad]
%   pose : [x; y; phi] of the gripper tip (arm frame, origin at joint 1)
%   P    : 2x5 joint positions [base, J2, J3, J4, tip] for plotting
    q = q(:).';
    if any(q < arm.qmin - 1e-9) || any(q > arm.qmax + 1e-9)
        warning('fkArm:limits', 'Joint angles outside range of motion.');
    end
    a = cumsum(q);                         % absolute link angles
    x = [0, cumsum(arm.L .* cos(a))];
    y = [0, cumsum(arm.L .* sin(a))];
    pose = [x(end); y(end); atan2(sin(a(end)), cos(a(end)))];
    P = [x; y];
end

function [qs, inLim] = ikArm(pose, psi, arm)
% IKARM  Inverse kinematics of the planar 4R arm.
%   pose  : [x; y; phi] target (arm frame, origin at joint 1)
%   psi   : vector of link-3 angles from horizontal to try [rad]
%   qs    : K x 4 joint solutions (up to 2 elbow branches per psi)
%   inLim : K x 1 logical, true if that solution is within joint limits
% Method: back off from the tip along the gripper to the wrist (joint 4),
% back off along link 3 (at angle psi) to joint 3, then solve the
% two-link arm (links 1-2) to joint 3 with the law of cosines.
    L = arm.L; x = pose(1); y = pose(2); phi = pose(3);
    psi = psi(:);
    xw = x - L(4)*cos(phi);   yw = y - L(4)*sin(phi);    % joint 4 (wrist)
    xe = xw - L(3)*cos(psi);  ye = yw - L(3)*sin(psi);   % joint 3, one per psi
    c2 = (xe.^2 + ye.^2 - L(1)^2 - L(2)^2) / (2*L(1)*L(2));
    keep = abs(c2) <= 1 + 1e-9;                           % reachable by links 1-2
    c2 = max(min(c2(keep), 1), -1);
    xe = xe(keep); ye = ye(keep); psi = psi(keep);
    qs = zeros(0, 4);
    for sgn = [1 -1]                                      % two elbow branches
        s2 = sgn * sqrt(1 - c2.^2);
        q2 = atan2(s2, c2);
        q1 = atan2(ye, xe) - atan2(L(2)*s2, L(1) + L(2)*c2);
        q3 = psi - q1 - q2;
        q4 = phi - psi + zeros(size(psi));
        qs = [qs; q1 q2 q3 q4]; %#ok<AGROW>
    end
    [qs, inLim] = fitToLimits(qs, arm.qmin, arm.qmax);
end

function [q, ok] = fitToLimits(q, qmin, qmax)
% Wrap each angle and pick the 2*pi-equivalent that lies within its limits.
% Works on K x n arrays; ok(k) is true if every joint of row k fits.
    tol = 1e-9;
    q = atan2(sin(q), cos(q));
    ok = true(size(q,1), 1);
    for i = 1:size(q,2)
        best = q(:,i);
        found = false(size(q,1), 1);
        for shift = [0 -2*pi 2*pi]
            c = q(:,i) + shift;
            fits = ~found & c >= qmin(i) - tol & c <= qmax(i) + tol;
            best(fits) = c(fits);
            found = found | fits;
        end
        q(:,i) = best;
        ok = ok & found;
    end
end

function q = chooseSolution(qs, arm, pref)
% CHOOSESOLUTION  Pick one pose from K x 4 candidates (all within limits).
%   pref = 1x4 joint angles -> least total joint travel from that pose
%   pref = 'margin'         -> farthest from the joint limits
%   pref = 'up' / 'down'    -> elbow (joint 2) up (q2 < 0) or down (q2 >= 0)
%                              if available, then farthest from the limits
    if isnumeric(pref)
        [~, j] = min(sum(abs(qs - pref(:).'), 2));
    else
        cand = (1:size(qs,1)).';
        if strcmpi(pref, 'up')
            b = find(qs(:,2) < 0);
        elseif strcmpi(pref, 'down')
            b = find(qs(:,2) >= 0);
        else
            b = [];
        end
        if ~isempty(b), cand = b; end
        span = arm.qmax - arm.qmin;
        margin = min(min(qs(cand,:) - arm.qmin, arm.qmax - qs(cand,:)) ./ span, [], 2);
        [~, k] = max(margin);
        j = cand(k);
    end
    q = qs(j,:);
end

function [q, msg] = ikBest(pose, arm, pref, mount)
% IKBEST  Best single IK solution for pose = [x; y; phi] (arm frame).
%   Tries arm.nPsi link-3 angles, keeps poses within the joint limits
%   (and clear of the ground and rover body if mount is given, [] to skip),
%   picks one with chooseSolution(pref), then repeats a finer search
%   around the chosen link-3 angle.
%   q is [] if nothing is usable, and msg says why.
    q = []; msg = '';
    [qs, inLim] = ikArm(pose, linspace(-pi, pi, arm.nPsi), arm);
    if isempty(qs)
        msg = 'out of reach (wrist point too far from joint 1)';
        return
    end
    qs = qs(inLim, :);
    if isempty(qs)
        msg = 'reachable, but every pose violates the joint limits at this gripper angle';
        return
    end
    if ~isempty(mount)
        qs = qs(clearOfObstacles(qs, arm, mount), :);
        if isempty(qs)
            msg = 'every pose within the limits collides with the ground or rover body';
            return
        end
    end
    q = chooseSolution(qs, arm, pref);

    % Fine search around the chosen link-3 angle (psi = q1 + q2 + q3)
    dpsi = 2*pi / (arm.nPsi - 1);
    [qf, inLf] = ikArm(pose, sum(q(1:3)) + linspace(-dpsi, dpsi, 41), arm);
    qf = qf(inLf, :);
    if ~isempty(mount) && ~isempty(qf)
        qf = qf(clearOfObstacles(qf, arm, mount), :);
    end
    q = chooseSolution([q; qf], arm, pref);
end

function [q, msg] = solveTargetArm(target, phi, arm, mount, pref)
% SOLVETARGETARM  IK for a ground-frame tip target [x, height] with gripper
%   angle phi, within the joint limits and clear of the ground and rover
%   body. pref chooses among the valid poses (see chooseSolution).
    pose = [target(1); target(2) - mount.h; phi];   % ground frame -> arm frame
    [q, msg] = ikBest(pose, arm, pref, mount);
end

function [ok, q] = canReachArm(target, arm, mount, nPhi)
% CANREACHARM  Can the tip reach target = [forward x, height above ground]
%   with any gripper angle? Collects usable poses over a sweep of phi and
%   returns the one farthest from the joint limits.
    if nargin < 4, nPhi = 73; end
    qAllOK = zeros(0, 4);
    for phi = linspace(-pi, pi, nPhi)
        pose = [target(1); target(2) - mount.h; phi];
        [qs, inLim] = ikArm(pose, linspace(-pi, pi, 120), arm);
        qs = qs(inLim, :);
        if ~isempty(qs)
            qAllOK = [qAllOK; qs(clearOfObstacles(qs, arm, mount), :)]; %#ok<AGROW>
        end
    end
    ok = ~isempty(qAllOK);
    q = [];
    if ok, q = chooseSolution(qAllOK, arm, 'margin'); end
end

function [M, C, G, dMdq] = dynArm(q, qd, arm)
% DYNARM  Equation-of-motion terms for the planar n-link arm:
%           M(q)*qdd + C(q,qd)*qd + G(q) = tau
%   q, qd : joint angles [rad] and rates [rad/s]
%   M     : n x n mass matrix
%   C     : n x n Coriolis/centrifugal matrix (Christoffel form)
%   G     : n x 1 gravity torques
%   dMdq  : n x n x n, dMdq(:,:,k) = dM/dq_k
    q = q(:); qd = qd(:); n = numel(q);
    a = cumsum(q);                         % absolute link angles
    T = tril(ones(n));                     % a = T*q

    % Last link's properties including the gripper payload
    [m, lc, I] = withPayload(arm);
    m = m(:); I = I(:);

    % r(i,j) = lever arm from joint j toward the COM of link i
    r = zeros(n);
    for i = 1:n
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
    dMa = zeros(n,n,n);
    for l = 1:n
        E = zeros(n); E(l,:) = 1; E(:,l) = E(:,l) - 1;   % delta_jl - delta_kl
        dMa(:,:,l) = -D .* sin(dA) .* E;
    end
    dMdq = zeros(n,n,n);
    for k = 1:n
        S = zeros(n);
        for l = 1:n
            S = S + T(l,k) * dMa(:,:,l);
        end
        dMdq(:,:,k) = T.' * S * T;
    end

    % Coriolis/centrifugal matrix from Christoffel symbols
    C = zeros(n);
    for i = 1:n
        for j = 1:n
            for k = 1:n
                C(i,j) = C(i,j) + 0.5*(dMdq(i,j,k) + dMdq(i,k,j) - dMdq(j,k,i)) * qd(k);
            end
        end
    end

    % Gravity: G = dV/dq, where V = -sum_i m_i * g . p_ci
    gx = arm.g(1); gy = arm.g(2);
    dVda = -(r.' * m) .* (-gx*sin(a) + gy*cos(a));
    G = T.' * dVda;
end

function [m, lc, I] = withPayload(arm)
% WITHPAYLOAD  Combine the last link (gripper) and the payload into one
%   rigid body: combined mass, COM distance from the last joint, and
%   inertia about the new COM (parallel-axis theorem).
    m = arm.m; lc = arm.lc; I = arm.I;
    n = numel(m);
    if isfield(arm, 'mp') && arm.mp > 0
        mn = m(n) + arm.mp;
        cn = (m(n)*lc(n) + arm.mp*arm.lp) / mn;
        I(n) = I(n) + m(n)*(lc(n) - cn)^2 + arm.Ip + arm.mp*(arm.lp - cn)^2;
        m(n) = mn;
        lc(n) = cn;
    end
end

function tau = invDynArm(q, qd, qdd, arm)
% INVDYNARM  Joint torques needed to produce motion (q, qd, qdd).
    [M, C, G] = dynArm(q, qd, arm);
    tau = M*qdd(:) + C*qd(:) + G;
end

function qdd = fwdDynArm(q, qd, tau, arm) %#ok<DEFNU>
% FWDDYNARM  Joint accelerations produced by torques tau (for simulation,
%   e.g. inside an ode45 right-hand side).
    [M, C, G] = dynArm(q, qd, arm);
    qdd = M \ (tau(:) - C*qd(:) - G);
end

function [ok, X, Y] = clearOfObstacles(q, arm, mount)
% CLEAROFOBSTACLES  For each row of q (K x n), true if every link stays at
%   least mount.clear above the ground and outside the rover keep-out box.
%   X, Y : K x (n+1) positions [base, joints..., tip] in the ground frame.
%   Links are straight, so the ground check only needs the joint points;
%   the box check samples 10 points along each link.
    K = size(q, 1); n = size(q, 2);
    a = cumsum(q, 2);
    X = [zeros(K,1), cumsum(arm.L .* cos(a), 2)];
    Y = mount.h + [zeros(K,1), cumsum(arm.L .* sin(a), 2)];
    ok = all(Y(:,2:end) >= mount.clear, 2);
    if ~isempty(mount.box)
        b = mount.box;
        if 0 > b(1) && 0 < b(2) && mount.h > b(3) && mount.h < b(4)
            error('Joint 1 is inside the rover keep-out box; raise mount.h or edit mount.box.');
        end
        for i = 1:n
            for s = 0.1:0.1:1
                xs = X(:,i) + s*(X(:,i+1) - X(:,i));
                ys = Y(:,i) + s*(Y(:,i+1) - Y(:,i));
                ok = ok & ~(xs > b(1) & xs < b(2) & ys > b(3) & ys < b(4));
            end
        end
    end
end

function env = reachEnvelopeArm(arm, mount, N)
% REACHENVELOPEARM  Sample the joint space (N values per joint, within the
%   limits) and keep the tip positions of collision-free poses.
%   env.tip : K x 2 reachable tip points [forward x, height above ground]
%   env.q   : K x 4 joint angles reaching them
%   Also prints the key reach numbers for this mount height.
    if nargin < 3, N = 30; end
    [Q1, Q2, Q3, Q4] = ndgrid(linspace(arm.qmin(1), arm.qmax(1), N), ...
                              linspace(arm.qmin(2), arm.qmax(2), N), ...
                              linspace(arm.qmin(3), arm.qmax(3), N), ...
                              linspace(arm.qmin(4), arm.qmax(4), N));
    q = [Q1(:) Q2(:) Q3(:) Q4(:)];
    clear Q1 Q2 Q3 Q4
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

function plotEnvelope(env, mount, col, name)
% Outline of the reachable region plus the mount point.
    kb = boundary(env.tip(:,1), env.tip(:,2), 0.9);
    plot(env.tip(kb,1), env.tip(kb,2), '-', 'Color', col, 'LineWidth', 2, ...
         'DisplayName', name);
    plot(0, mount.h, 's', 'Color', col, 'MarkerFaceColor', col, ...
         'HandleVisibility', 'off');
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
