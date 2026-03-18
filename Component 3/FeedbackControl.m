function  [twist, speeds, Xe, Xe_int, Je] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, Xe_int, current_state)
% This function computes the twist and the corresponding wheel and joint
% velocities needed to move the robot in the desired trajectory.
%
% This function implements feedforward + feedback control law in the end
% effector position.
%
% This function receives the inputs:
% X - current end effector configuration
% Xd - desired end effector position at current iteration (time step)
% Xd_next  - desired end effector position for next iteration 
% Kp - proportional gain
% ki - integral gain
% dt - time step
% Xe_int - integral error (error accumulated with time)
% current state
%
% This functions has outputs:
% twist
% speeds - wheel and joint velocities
% Xe - error
% Xe_int - integral error
% Je - jacobian matrix (both arm and chassis)

% Jacobian calculations
M0e = [1 0 0 0.033;...
       0 1 0 0;...
       0 0 1 0.6546;...
       0 0 0 1];

Blist = [0 0 1 0 0.033 0;...
         0 -1 0 -0.5076, 0, 0;...
         0 -1 0 -0.3526 0 0; ...
         0 -1 0 -0.2176 0 0; ...
         0 0 1 0 0 0]';

thetalist = current_state(4:8); %current joint angles

Jarm = JacobianBody(Blist, thetalist); %jacobian describing arm motion

T0e = FKinBody(M0e,Blist,thetalist'); 

Tb0 = [1 0 0 0.1662;...
       0 1 0 0;...
       0 0 1 0.0026;...
       0 0 0 1];

%chassis parameters
l = 0.47/2; 
w = 0.3/2;
r = 0.0475;

% H = 1/r*[-l-w 1 -1;...
%          l+w 1 1;...
%          l+w 1 -1;...
%          -l-w 1 1];

F = (r/4) * [ -1/(l+w)   1/(l+w)   1/(l+w)  -1/(l+w);
               1          1          1         1;
              -1          1         -1         1 ];

% 6x6 version of the F matrix (2 rows of 0 above and 1 row of zeros below
% to account for 0 rotation on both x and y axes and 0 translation on z
% axes)
F6 = [ zeros(2,4);
       F;
       zeros(1,4)];

% size(Ad_Teb)
% size(F6)

Jbase = Adjoint(TransInv(T0e)*TransInv(Tb0))*F6; %Jacobian for chassis motion

Je = [Jbase Jarm]; %combining both jacobians

%% calculating the outputs


twist_d = se3ToVec((1/dt)*MatrixLog6(TransInv(Xd)*Xd_next));

Xe = se3ToVec(MatrixLog6(TransInv(X)*Xd));

Xe_int = Xe_int + Xe*dt; %updating integral error

integral = Ki * (Xe_int); %itegral feedback

proportional = Kp * Xe; % proportional feedback

twist = Adjoint(TransInv(X)*Xd)*twist_d + proportional + integral; % feedforward + feedback control applied to the twist

speeds = pinv(Je,1e-3) * twist; %speeds extracted from twist

end