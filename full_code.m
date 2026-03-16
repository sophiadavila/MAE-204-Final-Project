clear; clc; close all;

Tse_initial = [0 0 1 0;
               0 1 0 0;
              -1 0 0 0.5;
               0 0 0 1];

Tsc_initial = [1 0 0 1;
               0 1 0 0;
               0 0 1 0.025;
               0 0 0 1];

Tsc_final = [0 1 0 0;
            -1 0 0 -1;
             0 0 1 0.025;
             0 0 0 1];

Tce_grasp = [-1 0 0 0;
              0 1 0 0;
              0 0 -1 0;
              0 0 0 1];

Tce_standoff = [-1 0 0 0;
                 0 1 0 0;
                 0 0 -1 0.1;
                 0 0 0 1];

Tb0 = [1 0 0 0.1662;...
       0 1 0 0;...
       0 0 1 0.0026;...
       0 0 0 1];

M0e = [1 0 0 0.033;...
       0 1 0 0;...
       0 0 1 0.6546;...
       0 0 0 1];

Blist = [0 0 1 0 0.033 0;...
         0 -1 0 -0.5076 0 0;...
         0 -1 0 -0.3526 0 0; ...
         0 -1 0 -0.2176 0 0; ...
         0 0 1 0 0 0]';

Kp = 7*eye(6);
Ki = 0*eye(6);

% Kp = 25*diag([2,2,2,2,2,2]);
% 
% Ki = 100*diag([1,1,1,1,1,1]);

dt = 0.01;

Xe_int = 0;

current_state = [0 0 0 0 0 0 -pi/4 0 0 0 0 0];

max_speed = 20;

traj = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, 1);

N = length(traj);
error_matrix = zeros(N-1,6);
rot_error = zeros(N-1,3);
trans_error = zeros(N-1,3);

configuration_matrix = zeros(N-1,13);

for i = 1:N-1
    
    phi = current_state(1); x = current_state(2); y = current_state(3);
    thetalist = current_state(4:8)';

    T0e = FKinBody(M0e,Blist,thetalist);
    Tsb = [cos(phi) -sin(phi) 0 x;...
       sin(phi) cos(phi) 0 y;...
       0 0 1 0.0963;...
       0 0 0 1];

    Tse = Tsb*Tb0*T0e;

    X = Tse;
    
    Xd = [traj(i,1), traj(i,2), traj(i,3), traj(i,10);
          traj(i,4), traj(i,5), traj(i,6), traj(i,11);
          traj(i,7), traj(i,8), traj(i,9), traj(i,12);
          0 0 0 1];

    Xd_next = [traj(i+1,1), traj(i+1,2), traj(i+1,3), traj(i+1,10);
          traj(i+1,4), traj(i+1,5), traj(i+1,6), traj(i+1,11);
          traj(i+1,7), traj(i+1,8), traj(i+1,9), traj(i+1,12);
          0 0 0 1];

    
    [twist, speeds, Xe, Xe_int] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, Xe_int, current_state);

    next_state = NextState(current_state, speeds, dt, max_speed);
    
    error_matrix(i, :) = Xe';
    rot_error(i,:) = Xe(1:3);%/max(Xe(1:3));
    trans_error(i,:) = Xe(4:6);%/max(Xe(4:6));

    configuration_matrix(i, :) = [current_state traj(i,end)];

    current_state = next_state;
end

t = (0:N-2)*dt;

figure(1)
plot(t, rot_error(:,1),'LineWidth',1.5)
hold on
plot(t, rot_error(:,2),'LineWidth',1.5)
plot(t, rot_error(:,3),'LineWidth',1.5)

figure(2)
plot(t, trans_error(:,1),'LineWidth',1.5)
hold on
plot(t, trans_error(:,2),'LineWidth',1.5)
plot(t, trans_error(:,3),'LineWidth',1.5)

xlabel('Time (s)')
ylabel('Normalized Error')
title('End-Effector Error vs Time')
grid on

figure(3);
plot(configuration_matrix)

writematrix(configuration_matrix,'full_code.csv')