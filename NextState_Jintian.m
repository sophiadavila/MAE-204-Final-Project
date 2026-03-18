% Next State function
function next_state = NextState(current_state, velocities, dt, max_speed)

% Current state - horizontal
% velocities - vertical

for i = 1:9
    if velocities(i)> max_speed
        velocities(i) = max_speed;
    end
end

joint_angles = current_state(4:8); %horizontal
joint_speeds = velocities(5:9); % vertical

new_joint_angles = joint_angles + joint_speeds' * dt; % horizontal

wheel_angles = current_state(9:12); % hotizontal
wheel_speeds = velocities(1:4); % vertical

d_wheel_angle = wheel_speeds * dt; % vertical

new_wheel_angles = wheel_angles + d_wheel_angle'; % horizontal

l = 0.47/2;
w = 0.3/2;
r = 0.0475;

chassis_twist = (r/4).*[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w); 1, 1, 1, 1; -1, 1, -1, 1]*d_wheel_angle;

q_0 = current_state(1:3); % horizontal

wbz = chassis_twist(1);
vbx = chassis_twist(2);
vby = chassis_twist(3);

if wbz == 0
    d_qb = [0; vbx; vby];
else
    d_qb = [wbz; (vbx*sin(wbz)+vby*(cos(wbz)-1))/wbz; (vby*sin(wbz)+vbx*(1-cos(wbz)))/wbz];    
end

d_q = [1 0 0; 0 cos(q_0(1)) -sin(q_0(1)); 0 sin(q_0(1)) cos(q_0(1))]*d_qb;

q_1 = q_0 + d_q'; % horizontal

next_state = [q_1(1) q_1(2) q_1(3) new_joint_angles(1) new_joint_angles(2) new_joint_angles(3) new_joint_angles(4) new_joint_angles(5) new_wheel_angles(1) new_wheel_angles(2) new_wheel_angles(3) new_wheel_angles(4)];

end