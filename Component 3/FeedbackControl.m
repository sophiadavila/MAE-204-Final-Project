function  [twist, speeds, Xe] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, current_state)

M0e = [1 0 0 0.033;...
       0 1 0 0;...
       0 0 1 0.6546;...
       0 0 0 1];

Blist = [0 0 1 0 0.033 0;...
         0 -1 0 -0.5076, 0, 0;...
         0 -1 0 -0.3526 0 0; ...
         0 -1 0 -0.2176 0 0; ...
         0 0 1 0 0 0]';

thetalist = current_state(4:8);

Jarm = JacobianBody(Blist, thetalist);

T0e = FKinBody(M0e,Blist,thetalist);

Tb0 = [1 0 0 0.1662;...
       0 1 0 0;...
       0 0 1 0.0026;...
       0 0 0 1];

l = 0.47/2;
w = 0.3/2;
r = 0.0475;

Tbe = Tb0*T0e;
Teb = inv(Tbe);

Ad_Teb = Adjoint(Teb);

H = 1/r*[-l-w 1 -1;...
         l+w 1 1;...
         l+w 1 -1;...
         -l-w 1 1];

Jbase = Ad_Teb(:,3:5)*pinv(H);

Je = [Jbase Jarm];

%% calculating the outputs

twist_d = (1/dt)*MatrixLog6(inv(Xd)*Xd_next);

twist_d = se3ToVec(twist_d);

Xe = MatrixLog6(inv(X)*Xd);

Xe = se3ToVec(Xe);

Ad = Adjoint(inv(X)*Xd);

integral = Ki * dt * Xe;
proportional = Kp * Xe;

twist = Ad*twist_d + proportional + integral;

speeds = pinv(Je) * twist;

end