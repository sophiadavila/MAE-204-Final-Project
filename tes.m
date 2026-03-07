%% Example Input Matrices for TrajectoryGenerator

clear; clc;

%% Initial End Effector Configuration (Tse_initial)
% End effector pointing downward above the robot
Tse_initial = [0 0 1 0;
               0 1 0 0;
              -1 0 0 0.5;
               0 0 0 1];

%% Cube Initial Configuration (Tsc_initial)
% Cube starts at (x,y,theta) = (1,0,0)
Tsc_initial = [1 0 0 1;
               0 1 0 0;
               0 0 1 0.025;
               0 0 0 1];

%% Cube Final Configuration (Tsc_final)
% Target location (x,y,theta) = (0,-1,-pi/2)
Tsc_final = [0 1 0 0;
            -1 0 0 -1;
             0 0 1 0.025;
             0 0 0 1];

%% End Effector Relative to Cube (Grasp)
% Gripper aligned to grasp cube
Tce_grasp = [-1 0 0 0;
              0 1 0 0;
              0 0 -1 0;
              0 0 0 1];

%% End Effector Standoff Configuration
% Hover 10 cm above cube before/after grasp
Tce_standoff = [-1 0 0 0;
                 0 1 0 0;
                 0 0 -1 0.1;
                 0 0 0 1];


traj = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, 1);


%% Output array to waypoint_array.csv
% waypoint_array.csv will be located in Matlab's current directory
writematrix(traj,'test_trajectory.csv')
