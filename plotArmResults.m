function plotArmResults(t,q_des,dq_des,ddq_des,Q_required,Q_inertia,Q_velocity,Q_gravity,p,L_gripper)

N = length(t);

q1 = q_des(:,1);
q2 = q_des(:,2);
q3 = q_des(:,3);

dq1 = dq_des(:,1);
dq2 = dq_des(:,2);
dq3 = dq_des(:,3);

ddq1 = ddq_des(:,1);
ddq2 = ddq_des(:,2);
ddq3 = ddq_des(:,3);

T1 = Q_required(:,1);
T2 = Q_required(:,2);
T3 = Q_required(:,3);

q1_deg = rad2deg(q1);
q2_deg = rad2deg(q2);
q3_deg = rad2deg(q3);

dq1_deg = rad2deg(dq1);
dq2_deg = rad2deg(dq2);
dq3_deg = rad2deg(dq3);

% JOINT ANGLES
figure
plot(t,q1_deg,'LineWidth',2)
hold on
plot(t,q2_deg,'LineWidth',2)
plot(t,q3_deg,'LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('Angle (deg)')
legend({'Joint 1','Joint 2','Joint 3'},'Location','best')
title('Joint Angles')

% JOINT VELOCITIES
figure
plot(t,dq1,'LineWidth',2)
hold on
plot(t,dq2,'LineWidth',2)
plot(t,dq3,'LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
legend({'Joint 1','Joint 2','Joint 3'},'Location','best')
title('Joint Angular Velocities')

% JOINT ACCELERATIONS
figure
plot(t,ddq1,'LineWidth',2)
hold on
plot(t,ddq2,'LineWidth',2)
plot(t,ddq3,'LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('Angular Acceleration (rad/s^2)')
legend({'Joint 1','Joint 2','Joint 3'},'Location','best')
title('Joint Angular Accelerations')

% REQUIRED MOTOR TORQUES
figure
plot(t,T1,'LineWidth',2)
hold on
plot(t,T2,'LineWidth',2)
plot(t,T3,'LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('Torque (N*m)')
legend({'Motor 1','Motor 2','Motor 3'},'Location','best')
title('Required Motor Torque')

% MOTOR 1 TORQUE BREAKDOWN
figure
plot(t,Q_inertia(:,1),'LineWidth',2)
hold on
plot(t,Q_velocity(:,1),'LineWidth',2)
plot(t,Q_gravity(:,1),'LineWidth',2)
plot(t,Q_required(:,1),'LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('Torque (N*m)')
legend({'Inertia','Velocity','Gravity','Total'},'Location','best')
title('Motor 1 Torque Breakdown')

% MOTOR 2 TORQUE BREAKDOWN
figure
plot(t,Q_inertia(:,2),'LineWidth',2)
hold on
plot(t,Q_velocity(:,2),'LineWidth',2)
plot(t,Q_gravity(:,2),'LineWidth',2)
plot(t,Q_required(:,2),'LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('Torque (N*m)')
legend({'Inertia','Velocity','Gravity','Total'},'Location','best')
title('Motor 2 Torque Breakdown')

% MOTOR 3 TORQUE BREAKDOWN
figure
plot(t,Q_inertia(:,3),'LineWidth',2)
hold on
plot(t,Q_velocity(:,3),'LineWidth',2)
plot(t,Q_gravity(:,3),'LineWidth',2)
plot(t,Q_required(:,3),'LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('Torque (N*m)')
legend({'Inertia','Velocity','Gravity','Total'},'Location','best')
title('Motor 3 Torque Breakdown')

% JOINT 1 POSITION AND VELOCITY
figure
yyaxis left
plot(t,q1_deg,'LineWidth',2)
ylabel('Joint 1 Position (deg)')
yyaxis right
plot(t,dq1_deg,'LineWidth',2)
ylabel('Joint 1 Velocity (deg/s)')
grid on
xlabel('Time (s)')
legend({'Position','Velocity'},'Location','best')
title('Joint 1 Position and Velocity')

% JOINT 2 POSITION AND VELOCITY
figure
yyaxis left
plot(t,q2_deg,'LineWidth',2)
ylabel('Joint 2 Position (deg)')
yyaxis right
plot(t,dq2_deg,'LineWidth',2)
ylabel('Joint 2 Velocity (deg/s)')
grid on
xlabel('Time (s)')
legend({'Position','Velocity'},'Location','best')
title('Joint 2 Position and Velocity')

% JOINT 3 POSITION AND VELOCITY
figure
yyaxis left
plot(t,q3_deg,'LineWidth',2)
ylabel('Joint 3 Position (deg)')
yyaxis right
plot(t,dq3_deg,'LineWidth',2)
ylabel('Joint 3 Velocity (deg/s)')
grid on
xlabel('Time (s)')
legend({'Position','Velocity'},'Location','best')
title('Joint 3 Position and Velocity')

% COMPARISON OF JOINT POSITIONS
figure
plot(t,q1_deg,'LineWidth',2)
hold on
plot(t,q2_deg,'LineWidth',2)
plot(t,q3_deg,'LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('Joint Position (deg)')
legend({'Joint 1','Joint 2','Joint 3'},'Location','best')
title('Comparison of Joint Positions')

% COMPARISON OF JOINT VELOCITIES
figure
plot(t,dq1_deg,'LineWidth',2)
hold on
plot(t,dq2_deg,'LineWidth',2)
plot(t,dq3_deg,'LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('Angular Velocity (deg/s)')
legend({'Joint 1','Joint 2','Joint 3'},'Location','best')
title('Comparison of Joint Velocities')

% MAXIMUM TORQUE CONTRIBUTIONS
T1_inertia_max = max(abs(Q_inertia(:,1)));
T1_velocity_max = max(abs(Q_velocity(:,1)));
T1_gravity_max = max(abs(Q_gravity(:,1)));

T2_inertia_max = max(abs(Q_inertia(:,2)));
T2_velocity_max = max(abs(Q_velocity(:,2)));
T2_gravity_max = max(abs(Q_gravity(:,2)));

T3_inertia_max = max(abs(Q_inertia(:,3)));
T3_velocity_max = max(abs(Q_velocity(:,3)));
T3_gravity_max = max(abs(Q_gravity(:,3)));

TorqueParts = [T1_inertia_max T1_velocity_max T1_gravity_max;
               T2_inertia_max T2_velocity_max T2_gravity_max;
               T3_inertia_max T3_velocity_max T3_gravity_max];

figure
bar(TorqueParts)
grid on
xlabel('Motor')
ylabel('Maximum Torque Contribution (N*m)')
xticklabels({'Motor 1','Motor 2','Motor 3'})
legend({'Inertia M(q)qdd','Velocity C(q,qd)qd','Gravity G(q)'},'Location','best')
title('Maximum Motor Torque Contributions')

% END EFFECTOR POSITION
x_end = zeros(N,1);
y_end = zeros(N,1);

for i = 1:N

    x1 = p.L1*cos(q1(i));
    y1 = p.L1*sin(q1(i));

    x2 = x1 + p.L2*cos(q2(i));
    y2 = y1 + p.L2*sin(q2(i));

    x_end(i) = x2 + L_gripper*cos(q3(i));
    y_end(i) = y2 + L_gripper*sin(q3(i));

end

figure
plot(t,x_end,'LineWidth',2)
hold on
plot(t,y_end,'LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('Position (m)')
legend({'X Position','Y Position'},'Location','best')
title('End Effector Position')

% END EFFECTOR TRAJECTORY
figure
plot(x_end,y_end,'LineWidth',2)
hold on
plot(x_end(1),y_end(1),'o','MarkerSize',8,'LineWidth',2)
plot(x_end(end),y_end(end),'s','MarkerSize',8,'LineWidth',2)
axis equal
grid on
xlabel('X Position (m)')
ylabel('Y Position (m)')
legend({'End Effector Path','Start','Finish'},'Location','best')
title('End Effector Trajectory')

end