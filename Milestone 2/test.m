clear; clc

current_state = [0 0 0 0 0 0 0 0 0 0 0 0];
dt = 0.01;
max_speed = 20;

speeds_forward = [0 0 0 0 0 5 5 5 5];
speeds_rotate = [0 0 0 0 0 -20 20 20 -20];
speeds_sideways = [0 degtorad(-20) degtorad(-30) degtorad(-90) degtorad(-90) -5 5 -5 5];

traj= zeros(400,13);

state = current_state;
for i = 1:100
    state = NextState(state, speeds_forward, dt, max_speed);
    traj(i,:) = [state 0];
end

for i = 101:300
    state = NextState(state, speeds_rotate, dt, max_speed);
    traj(i,:) = [state 0];
end

for i = 301:400
    state = NextState(state, speeds_sideways, dt, max_speed);
    traj(i,:) = [state 0];
end


writematrix(traj,'test.csv')