%% Initialization:
close all
clear all

% Refrence Trajectory:
R = 2 ;     % trajectory radius (m)

% Kinematic Controller Gains are determined by gain scheduling in Simulink
% in the Kinematic Controller subsystem


% Model Parameters:
act_theta1 = 0.021281;
act_theta2 = 0.02017;
act_theta3 = 1.2e-4;
act_theta4 = 1.0236;
act_theta5 = 1.0438e-4;
act_theta6 = 1.00271;
act_theta_m = [act_theta1 act_theta2 act_theta3 act_theta4 act_theta5 act_theta6];

% Initial Robot Pose:
x_init = 2;
y_init = 0;
phi_init = pi/2;

% Controller Parameters Adaptation:
Gamma = eye(8);
theta0_c = ones(1,8);

% RLS for Model Parameters
P0 = eye(6);
alpha = 1;
theta0_m = ones(1,6);

% Reference Model:
Cm = eye(2);
Bm = eye(2);
am = 5;
Am = -am*eye(2);
Dm = zeros(2,2);

Q = eye(2);
P_lyap = lyap(Am,Q);


%% No Disturbance:
disturbance = 0;

sim kin_lin_dyn_FSMRAC_lin2

figure(1)
plot(x_R,y_R,'-r')
hold on
plot(xout,yout,'--b')
plot(xout_mrac,yout_mrac,'-.g')
title('Reference Trajectory, Robot Trajectory and Reference Model Trajectory')
xlabel('x (m)')
ylabel('y (m)')
axis([-2.5 3.8 -2.5 3.8])

figure(2)
plot(tout,phi_ang_R,'-r')
hold on
plot(tout,phi_ang_out,'--b')
plot(tout,phi_ang_mrac,'-.g')
title('Reference Heading Angle, Robot Heading Angle and Reference Model Heading Angle')
xlabel('Time (s)')
ylabel('Heading Angle (rad)')

figure(3)
plot(tout,vc,'-r')
hold on
plot(tout,v,'--b')
plot(tout,vm,'-.g')
title('Reference Linear Velocity, Robot Linear Velocity and Reference Model Linear Velocity')
xlabel('Time (s)')
ylabel('Linear Velocity (m/s)')

figure(4)
plot(tout,wc,'-r')
hold on
plot(tout,w,'--b')
plot(tout,wm,'-.g')
title('Reference Angular Velocity, Robot Angular Velocity and Reference Model Angular Velocity')
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')

figure(5)
plot(tout,v_ref)
title('Linear Velocity Control Input')
xlabel('Time (s)')
ylabel('Linear Velocity Control Input vref (m/s)')

figure(6)
plot(tout,w_ref)
title('Angular Velocity Control Input')
xlabel('Time (s)')
ylabel('Angular Velocity Control Input wref (rad/s)')

figure(7)
plot(tout,theta_m)
title('Estimated Model Parameters')
xlabel('Time (s)')
ylabel('Estimated Model Parameters')

figure(8)
plot(tout,theta_c)
title('Controller Parameters')
xlabel('Time (s)')
ylabel('Controller Parameters')


%% With Disturbance:
disturbance = 1;

sim kin_lin_dyn_FSMRAC_lin2

figure(9)
plot(x_R,y_R,'-r')
hold on
plot(xout,yout,'--b')
plot(xout_mrac,yout_mrac,'-.g')
title('Reference, Robot and Reference Model Trajectories with Disturbances on vref and wref')
xlabel('x (m)')
ylabel('y (m)')
axis([-2.5 3.8 -2.5 3.8])

figure(10)
plot(tout,phi_ang_R,'-r')
hold on
plot(tout,phi_ang_out,'--b')
plot(tout,phi_ang_mrac,'-.g')
title('Reference, Robot and Reference Model Heading Angles with Disturbances on vref and wref')
xlabel('Time (s)')
ylabel('Heading Angle (rad)')

figure(11)
plot(tout,vc,'-r')
hold on
plot(tout,v,'--b')
plot(tout,vm,'-.g')
title('Reference , Robot and Reference Model Linear Velocities with Disturbances on vref and wref')
xlabel('Time (s)')
ylabel('Linear Velocity (m/s)')

figure(12)
plot(tout,wc,'-r')
hold on
plot(tout,w,'--b')
plot(tout,wm,'-.g')
title('Reference, Robot and Reference Model Angular Velocities with Disturbances on vref and wref')
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')

figure(13)
plot(tout,v_ref)
title('Linear Velocity Control Input with Disturbances on vref and wref')
xlabel('Time (s)')
ylabel('Linear Velocity Control Input vref (m/s)')

figure(14)
plot(tout,w_ref)
title('Angular Velocity Control Input with Disturbances on vref and wref')
xlabel('Time (s)')
ylabel('Angular Velocity Control Input wref (rad/s)')

figure(15)
plot(tout,theta_m)
title('Estimated Model Parameters with Disturbances on vref and wref')
xlabel('Time (s)')
ylabel('Estimated Model Parameters')

figure(16)
plot(tout,theta_c)
title('Controller Parameters with Disturbances on vref and wref')
xlabel('Time (s)')
ylabel('Controller Parameters')

