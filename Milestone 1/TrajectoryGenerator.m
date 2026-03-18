% code for TrajectoryGenerator function
% This function generates the reference (desired) trajectory for the end-effector frame {e}.
% Inputs are 
% Output is 
function traj = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k)

    t = 2;
    N = t*k/(0.01);

    % move 1
    Tse_standoff_initial = Tsc_initial*Tce_standoff;
    traj1 = ScrewTrajectory(Tse_initial, Tse_standoff_initial, t, N, 5);
    traj1_matrix = zeros(N,13);
    gripper_state = 0;

    for i = 1:N
        traj1_matrix(i,:) = [traj1{i}(1,1),traj1{i}(1,2), traj1{i}(1,3), traj1{i}(2,1), traj1{i}(2,2), traj1{i}(2,3), traj1{i}(3,1), traj1{i}(3,2), traj1{i}(3,3), traj1{i}(1,4), traj1{i}(2,4), traj1{i}(3,4), gripper_state];
    end
    
    % move 2
    Tse_grasp = Tsc_initial* Tce_grasp;
    traj2 = ScrewTrajectory(Tse_standoff_initial, Tse_grasp, t, N, 5);
    traj2_matrix = zeros(N,13);

    for i = 1:N
        traj2_matrix(i,:) = [traj2{i}(1,1),traj2{i}(1,2), traj2{i}(1,3), traj2{i}(2,1), traj2{i}(2,2), traj2{i}(2,3), traj2{i}(3,1), traj2{i}(3,2), traj2{i}(3,3), traj2{i}(1,4), traj2{i}(2,4), traj2{i}(3,4), gripper_state];
    end
    
    % move 3
    traj3 = ScrewTrajectory(Tse_grasp, Tse_grasp, t, N, 5);
    traj3_matrix = zeros(N,13);
    gripper_state = 1;

    for i = 1:N
        traj3_matrix(i,:) = [traj3{i}(1,1),traj3{i}(1,2), traj3{i}(1,3), traj3{i}(2,1), traj3{i}(2,2), traj3{i}(2,3), traj3{i}(3,1), traj3{i}(3,2), traj3{i}(3,3), traj3{i}(1,4), traj3{i}(2,4), traj3{i}(3,4), gripper_state];
    end
    
    % move 4
    traj4 = ScrewTrajectory(Tse_grasp, Tse_standoff_initial, t, N, 5);
    traj4_matrix = zeros(N,13);

    for i = 1:N
        traj4_matrix(i,:) = [traj4{i}(1,1),traj4{i}(1,2), traj4{i}(1,3), traj4{i}(2,1), traj4{i}(2,2), traj4{i}(2,3), traj4{i}(3,1), traj4{i}(3,2), traj4{i}(3,3), traj4{i}(1,4), traj4{i}(2,4), traj4{i}(3,4), gripper_state];
    end

    % move 5
    Tse_standoff_final = Tsc_final*Tce_standoff;
    traj5 = ScrewTrajectory(Tse_standoff_initial, Tse_standoff_final, t, N, 5);
    traj5_matrix = zeros(N,13);
    
    for i = 1:N
        traj5_matrix(i,:) = [traj5{i}(1,1),traj5{i}(1,2), traj5{i}(1,3), traj5{i}(2,1), traj5{i}(2,2), traj5{i}(2,3), traj5{i}(3,1), traj5{i}(3,2), traj5{i}(3,3), traj5{i}(1,4), traj5{i}(2,4), traj5{i}(3,4), gripper_state];
    end

    % move 6
    Tse_final = Tsc_final*Tce_grasp;
    traj6 = ScrewTrajectory(Tse_standoff_final, Tse_final, t, N, 5);
    traj6_matrix = zeros(N,13);

    for i = 1:N
        traj6_matrix(i,:) = [traj6{i}(1,1),traj6{i}(1,2), traj6{i}(1,3), traj6{i}(2,1), traj6{i}(2,2), traj6{i}(2,3), traj6{i}(3,1), traj6{i}(3,2), traj6{i}(3,3), traj6{i}(1,4), traj6{i}(2,4), traj6{i}(3,4), gripper_state];
    end

    % move 7
    traj7 = ScrewTrajectory(Tse_final, Tse_final, t, N, 5);
    traj7_matrix = zeros(N,13);
    gripper_state = 0;

    for i = 1:N
        traj7_matrix(i,:) = [traj7{i}(1,1),traj7{i}(1,2), traj7{i}(1,3), traj7{i}(2,1), traj7{i}(2,2), traj7{i}(2,3), traj7{i}(3,1), traj7{i}(3,2), traj7{i}(3,3), traj7{i}(1,4), traj7{i}(2,4), traj7{i}(3,4), gripper_state];
    end

    % move 8
    traj8 = ScrewTrajectory(Tse_final, Tse_standoff_final, t, N, 5);
    traj8_matrix = zeros(N,13);
    
    for i = 1:N
        traj8_matrix(i,:) = [traj8{i}(1,1),traj8{i}(1,2), traj8{i}(1,3), traj8{i}(2,1), traj8{i}(2,2), traj8{i}(2,3), traj8{i}(3,1), traj8{i}(3,2), traj8{i}(3,3), traj8{i}(1,4), traj8{i}(2,4), traj8{i}(3,4), gripper_state];
    end

    % matrix with all trajectories
    traj = [traj1_matrix; traj2_matrix; traj3_matrix; traj4_matrix; traj5_matrix; traj6_matrix; traj7_matrix; traj8_matrix];
end
