function [x_target,y_target,q1_final,q2_final,q3_final,L_gripper] = selectGripperTarget1(q1_0,q2_0,q3_0,p)

% SETTINGS
L_gripper = 0.15; % m
q3_final = deg2rad(90); % Absolute gripper angle (rad)
elbow = -1; % Relative elbow angle sign: 1 or -1

% REACHABLE GRIPPER-TIP POSITIONS FOR THE FINAL GRIPPER ANGLE
% Geometric reach only; joint limits and collisions are not included.
Rmax = p.L1 + p.L2;
Rmin = abs(p.L1 - p.L2);
gx = L_gripper*cos(q3_final);
gy = L_gripper*sin(q3_final);
phi = linspace(0,2*pi,500);

% INITIAL ARM POSITION (all angles are absolute)
x1 = p.L1*cos(q1_0);
y1 = p.L1*sin(q1_0);
x2 = x1 + p.L2*cos(q2_0);
y2 = y1 + p.L2*sin(q2_0);
x3 = x2 + L_gripper*cos(q3_0);
y3 = y2 + L_gripper*sin(q3_0);

fig = figure('Name','Select Gripper Target','NumberTitle','off');
ax = axes('Parent',fig);
hold(ax,'on')
fill(ax,gx + Rmax*cos(phi),gy + Rmax*sin(phi),[0.85 0.85 0.85], ...
    'FaceAlpha',0.35,'EdgeColor','k','LineStyle','--','LineWidth',1.5)
if Rmin > 0
    plot(ax,gx + Rmin*cos(phi),gy + Rmin*sin(phi),'k--','LineWidth',1.5)
end
plot(ax,[0 x1 x2],[0 y1 y2],'-o','LineWidth',3)
plot(ax,[x2 x3],[y2 y3],'-o','LineWidth',3,'Color',[0.2 0.6 0.2])
plot(ax,0,0,'s','MarkerSize',10,'MarkerFaceColor','k','MarkerEdgeColor','k')
axis(ax,'equal')
grid(ax,'on')
xlabel(ax,'X position (m)')
ylabel(ax,'Y position (m)')
title(ax,{'Select the GRIPPER TIP position','Inside outer boundary; outside inner boundary if shown'})
margin = 0.1*Rmax;
xlim(ax,[min(gx-Rmax,min([0 x1 x2 x3]))-margin max(gx+Rmax,max([0 x1 x2 x3]))+margin])
ylim(ax,[min(gy-Rmax,min([0 y1 y2 y3]))-margin max(gy+Rmax,max([0 y1 y2 y3]))+margin])

method = menu('Select target position','Click on graph','Enter X and Y');
if method == 0
    if isgraphics(fig), close(fig), end
    error('Arm:TargetCancelled','Target selection cancelled.')
end

while true
    if ~isgraphics(fig)
        error('Arm:TargetCancelled','Target selection cancelled.')
    end
    if method == 1
        figure(fig)
        axes(ax)
        try
            [x_target,y_target,button] = ginput(1);
        catch ME
            if ~isgraphics(fig)
                error('Arm:TargetCancelled','Target selection cancelled.')
            end
            rethrow(ME)
        end
        if isempty(button) || button == 27
            close(fig)
            error('Arm:TargetCancelled','Target selection cancelled.')
        end
        if button ~= 1
            continue
        end
    else
        answer = inputdlg({'X position (m):','Y position (m):'}, ...
            'Gripper Tip Target',[1 35],{'0.5','0.4'});
        if isempty(answer)
            close(fig)
            error('Arm:TargetCancelled','Target selection cancelled.')
        end
        x_target = str2double(answer{1});
        y_target = str2double(answer{2});
        if ~isreal([x_target y_target]) || any(~isfinite([x_target y_target]))
            uiwait(warndlg('Enter finite, real numbers for X and Y.','Invalid Target'))
            continue
        end
    end

    % Remove the gripper offset to find the Motor 3 position.
    xw = x_target - gx;
    yw = y_target - gy;
    r = hypot(xw,yw);
    tol = 1e-10*max(1,Rmax);
    if r > Rmax + tol || r < Rmin - tol
        uiwait(warndlg('Target is outside the reachable region. Select another position.','Unreachable Target'))
        continue
    end

    % INVERSE KINEMATICS
    c2 = (xw^2 + yw^2 - p.L1^2 - p.L2^2)/(2*p.L1*p.L2);
    c2 = max(-1,min(1,c2)); % Prevent roundoff errors at the boundary.
    q2_relative = elbow*acos(c2);
    if r <= tol && Rmin <= tol
        q1_final = q1_0; % Folded equal-length links: shoulder angle is free.
    else
        q1_final = atan2(yw,xw) - atan2(p.L2*sin(q2_relative),p.L1 + p.L2*cos(q2_relative));
    end
    q2_final = q1_final + q2_relative; % Convert to absolute Link 2 angle.

    % Equivalent angles nearest the initial angles avoid unnecessary turns.
    q1_final = q1_0 + atan2(sin(q1_final-q1_0),cos(q1_final-q1_0));
    q2_final = q2_0 + atan2(sin(q2_final-q2_0),cos(q2_final-q2_0));
    q3_final = q3_0 + atan2(sin(q3_final-q3_0),cos(q3_final-q3_0));
    break
end

% SHOW THE SELECTED FINAL ARM POSITION
xf1 = p.L1*cos(q1_final);
yf1 = p.L1*sin(q1_final);
xf2 = xf1 + p.L2*cos(q2_final);
yf2 = yf1 + p.L2*sin(q2_final);
plot(ax,[0 xf1 xf2],[0 yf1 yf2],'k-o','LineWidth',2)
plot(ax,[xf2 x_target],[yf2 y_target],'g-o','LineWidth',2)
plot(ax,x_target,y_target,'gx','MarkerSize',14,'LineWidth',3)
text(ax,x_target+0.02,y_target+0.02,sprintf('(%.2f, %.2f) m',x_target,y_target))
title(ax,sprintf('Selected gripper tip: X = %.3f m, Y = %.3f m',x_target,y_target))
drawnow

end
