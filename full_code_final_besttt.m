clear; clc; close all;

%% Transformation matrices for our desired positions:
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

c = sqrt(2)/2;

Tce_grasp = [-c   0   c   0;
              0   1   0   0;
             -c   0  -c   0;
              0   0   0   1];

% Tce_grasp = [-1 0 0 0;
%               0 1 0 0;
%               0 0 -1 0;
%               0 0 0 1];

Tce_standoff = [-c 0 c 0;
                 0 1 0 0;
                 -c 0 -c 0.1;
                 0 0 0 1];

% Tce_standoff = [-1 0 0 0;
%                  0 1 0 0;
%                  0 0 -1 0.1;
%                  0 0 0 1];

%% Generating the trajectory
traj = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, 1);

%% Important matrices for later calculations (given in the Coppeliasim website - scene 6)
Tb0 = [1 0 0 0.1662;
       0 1 0 0;
       0 0 1 0.0026;
       0 0 0 1];

M0e = [1 0 0 0.033;
       0 1 0 0;
       0 0 1 0.6546;
       0 0 0 1];

Blist = [0 0 1 0 0.033 0;
         0 -1 0 -0.5076 0 0;
         0 -1 0 -0.3526 0 0;
         0 -1 0 -0.2176 0 0;
         0 0 1 0 0 0]';

%% Other inputs and initializations
dt = 0.01;
max_speed = 1000;

 % jlim = [-2.9  2.9;
 %  -1.5  1.5;
 %  -2.6  2.6;
 %  -1.8 -0.2;   % avoids singularity
 %  -2.9  2.9];

 current_state = [0 0 0 0 0 -pi/2 -pi/4 0 0 0 0 0]; %might not meet requirement
%current_state = [pi/6 0.3 0.3 0 0 0 0 0 0 0 0 0];

N = length(traj);

Xe_int = 0;

% memory allocation for error matrices
error_matrix = zeros(N-1,6);
rot_error = zeros(N-1,3); %for plotting
trans_error = zeros(N-1,3); %for plotting

mu_w = zeros(N-1,1);
mu_v = zeros(N-1,1);

configuration_matrix = zeros(N-1,13);

%% Controller parameters 
Kp = 1.5*eye(6);
Ki = 0*eye(6);

%% Main loop
for i = 1:N-1
    
    phi = current_state(1); x = current_state(2); y = current_state(3);
    thetalist = current_state(4:8)'; 

    T0e = FKinBody(M0e,Blist,thetalist);
    Tsb = [cos(phi) -sin(phi) 0 x;
           sin(phi) cos(phi) 0 y;
           0 0 1 0.0963;
           0 0 0 1];

    Tse = Tsb*Tb0*T0e;
    
    % setting the inputs for Feedback Control function
    X = Tse;
    
    Xd = [traj(i,1:3) traj(i,10);
          traj(i,4:6) traj(i,11);
          traj(i,7:9) traj(i,12);
          0 0 0 1];

    Xd_next = [traj(i+1,1:3) traj(i+1,10);
               traj(i+1,4:6) traj(i+1,11);
               traj(i+1,7:9) traj(i+1,12);
               0 0 0 1];

    [twist, speeds, Xe, Xe_int, Je] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, Xe_int, current_state);
    
    next_state = NextState(current_state, speeds, dt, max_speed);
    %next_state(4:8) = limit_joints(next_state(4:8),jlim);
    
    error_matrix(i,:) = Xe';
    rot_error(i,:) = Xe(1:3);
    trans_error(i,:) = Xe(4:6);

    Jw = Je(1:3,:);
    Jv = Je(4:6,:);

    Aw = Jw*Jw';
    Av = Jv*Jv';
    mu_w(i) = (sqrt(max(eig(Aw)))) / (sqrt(min(eig(Aw))));
    mu_v(i) = (sqrt(max(eig(Av)))) / (sqrt(min(eig(Av))));

    configuration_matrix(i,:) = [current_state traj(i,end)];

    current_state = next_state;
end

%% Plotting everything in one figure with subplots
t = (0:N-2)*dt;

figure(4);

% Rotational error subplot
subplot(3,1,1)
plot(t, rot_error(:,1),'LineWidth',1.5)
hold on
plot(t, rot_error(:,2),'LineWidth',1.5)
plot(t, rot_error(:,3),'LineWidth',1.5)
plot(t, trans_error(:,1),'LineWidth',1.5)
plot(t, trans_error(:,2),'LineWidth',1.5)
plot(t, trans_error(:,3),'LineWidth',1.5)
xlabel('Time (s)')
ylabel('Error')
title('Rotational Error (\omega)')
legend('\omega_x','\omega_y','\omega_z','v_x','v_y','v_z')
grid on



% Manipulability factor mu_w
subplot(3,1,2)
plot(t, mu_w,'LineWidth',1.5)
xlabel('Time (s)')
ylabel('\mu_\omega')
title('Rotational Manipulability')
grid on

% Manipulability factor mu_v
subplot(3,1,3)
plot(t, mu_v,'LineWidth',1.5)
xlabel('Time (s)')
ylabel('\mu_v')
title('Translational Manipulability')
grid on


% Save configuration
writematrix(configuration_matrix,'full_code.csv')
