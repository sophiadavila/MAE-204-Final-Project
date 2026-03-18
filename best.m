clear; clc; close all;

%% Transformation matrices for our desired positions:
% these can be changed to achieve different motion
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

%% Generating the trajectory
traj = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, 1);

%% Important matrices for later calculations (given in the CoppeliaSim website - scene 6)
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

%% Other inputs and initializations
dt = 0.01;
max_speed = 1000;

%current_state = [0 1 0 0 0 0 0 0 0 0 0 0]; %different initial state tests 
current_state = [pi/4 0.3 0.3 0 0 0 0 0 0 0 0 0]; %current state with at least 30 degree orientation error and 0.2m position error
%current_state = [pi/3 0.5 -0.3 0.2 -0.5 0.3 0 0 0 0 0 0]; %different initial state tests 

N = length(traj);

% error initialization to 0
Xe_int = 0;

% memory allocation for error matrices
error_matrix = zeros(N-1,6);
rot_error = zeros(N-1,3); %for plotting
trans_error = zeros(N-1,3); %for plotting

% manipulability factors memory alocation
mu_w = zeros(N-1,1);
mu_v = zeros(N-1,1);

% memory allocation for configuration matrix
configuration_matrix = zeros(N-1,13);

%% Controller parameters 
Kp = 20*eye(6);
Ki = 0.01*eye(6);

%% for loop iterating through steps of the generated trajectory
for i = 1:N-1
    
    phi = current_state(1); x = current_state(2); y = current_state(3); %extracting chassis values from current state
    thetalist = current_state(4:8)'; %extracting joint values from current state

    %matrices used to find Tse (current config)
    T0e = FKinBody(M0e,Blist,thetalist);
    Tsb = [cos(phi) -sin(phi) 0 x;...
       sin(phi) cos(phi) 0 y;...
       0 0 1 0.0963;...
       0 0 0 1];

    Tse = Tsb*Tb0*T0e;

    % setting the inputs for Feedback Control function
    X = Tse;
    
    Xd = [traj(i,1), traj(i,2), traj(i,3), traj(i,10);
          traj(i,4), traj(i,5), traj(i,6), traj(i,11);
          traj(i,7), traj(i,8), traj(i,9), traj(i,12);
          0 0 0 1];

    Xd_next = [traj(i+1,1), traj(i+1,2), traj(i+1,3), traj(i+1,10);
          traj(i+1,4), traj(i+1,5), traj(i+1,6), traj(i+1,11);
          traj(i+1,7), traj(i+1,8), traj(i+1,9), traj(i+1,12);
          0 0 0 1];

    
    %running feedback control
    [twist, speeds, Xe, Xe_int, Je] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, Xe_int, current_state);
    
    %getting the next state
    next_state = NextState(current_state, speeds, dt, max_speed);
    
    %storing the error 
    error_matrix(i, :) = Xe';
    rot_error(i,:) = Xe(1:3);%/max(Xe(1:3));
    trans_error(i,:) = Xe(4:6);%/max(Xe(4:6));
    
    %calculating and storing the manipulability factors
    Jw = Je(1:3,:);
    Jv = Je(4:6,:);

    mu_w(i) = sqrt(abs(det(Jw*Jw')));
    mu_v(i) = sqrt(abs(det(Jv*Jv')));

    %storing the configuration of each step
    configuration_matrix(i, :) = [current_state traj(i,end)];

    %updating state
    current_state = next_state;
end

%plotting 
t = (0:N-2)*dt; %time vector

figure(1) %plotting rotational error (w)
plot(t, rot_error(:,1),'LineWidth',1.5)
hold on
plot(t, rot_error(:,2),'LineWidth',1.5)
plot(t, rot_error(:,3),'LineWidth',1.5)

xlabel('Time (s)')
ylabel('Normalized Error')
title('End-Effector Error vs Time')
legend('\omega_x','\omega_y', '\omega_z')
grid on

figure(2) %ploting translational error (v)
plot(t, trans_error(:,1),'LineWidth',1.5)
hold on
plot(t, trans_error(:,2),'LineWidth',1.5)
plot(t, trans_error(:,3),'LineWidth',1.5)

xlabel('Time (s)')
ylabel('Normalized Error')
title('End-Effector Error vs Time')
legend('v_x','v_y', 'v_z')
grid on

% plotting manipulability factors
figure(3);
plot(t, mu_w, 'LineWidth',1.5)
hold on
plot(t, mu_v, 'LineWidth',1.5)

xlabel('Time (s)')
ylabel('Manipulability')
title('Manipulability vs Time')
legend('\mu_{\omega}','\mu_v')
grid on

% figure(4);
% plot(configuration_matrix)

writematrix(configuration_matrix,'best.csv') %saving configuration into a csv file