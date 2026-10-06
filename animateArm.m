function animateArm(q_des,t,p,numberRuns,L_gripper)

N=length(t);

if nargin<5
L_gripper=0.15;
end

figure
hold on

armPlot=plot(nan,nan,'-o','LineWidth',3,'MarkerSize',8);
gripperPlot=plot(nan,nan,'-o','LineWidth',3,'MarkerSize',6,...
    'Color',[0.2 0.6 0.2]);

labelM1=text(0,0,'Motor 1','FontSize',10,'FontWeight','bold');
labelL1=text(0,0,'Link 1','FontSize',10);
labelM2=text(0,0,'Motor 2','FontSize',10,'FontWeight','bold');
labelL2=text(0,0,'Link 2','FontSize',10);
labelM3=text(0,0,'Motor 3','FontSize',10,'FontWeight','bold');
labelGrip=text(0,0,'Gripper','FontSize',10);

axis equal
grid on
xlabel('X Position (m)')
ylabel('Y Position (m)')
title('Robotic Arm Motion')

R=p.L1+p.L2+L_gripper;
margin=0.1;
xlim([-R-margin R+margin])
ylim([-R-margin R+margin])

for run=1:numberRuns

for direction=1:2

if direction==1
indices=1:N;
motion='Forward';
else
indices=N:-1:1;
motion='Returning';
end

for j=1:length(indices)

k=indices(j);

if ~isgraphics(armPlot)
return
end

% GENERALIZED COORDINATES
q1=q_des(k,1);
q2=q_des(k,2);
q3=q_des(k,3);

% MOTOR 1: FIXED BASE
x0=0;
y0=0;

% MOTOR 2: END OF LINK 1
x1=x0+p.L1*cos(q1);
y1=y0+p.L1*sin(q1);

% MOTOR 3: END OF LINK 2
x2=x1+p.L2*cos(q2);
y2=y1+p.L2*sin(q2);

% GRIPPER TIP
x3=x2+L_gripper*cos(q3);
y3=y2+L_gripper*sin(q3);

% UPDATE ARM
armPlot.XData=[x0 x1 x2];
armPlot.YData=[y0 y1 y2];

gripperPlot.XData=[x2 x3];
gripperPlot.YData=[y2 y3];

% UPDATE LABELS
labelM1.Position=[x0+0.02 y0+0.02 0];
labelL1.Position=[(x0+x1)/2+0.02 (y0+y1)/2+0.02 0];
labelM2.Position=[x1+0.02 y1+0.02 0];
labelL2.Position=[(x1+x2)/2+0.02 (y1+y2)/2+0.02 0];
labelM3.Position=[x2+0.02 y2+0.02 0];
labelGrip.Position=[x3+0.02 y3+0.02 0];

title(sprintf('Run %d of %d - %s - Time %.2f s',...
    run,numberRuns,motion,t(k)))

drawnow

if j<length(indices)
pause(abs(t(indices(j+1))-t(k)))
end

end
end
end
end