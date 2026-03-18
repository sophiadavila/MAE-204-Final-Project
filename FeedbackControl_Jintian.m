function  [twist, speeds, Xe, Xe_int, Je] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, Xe_int, current_state)

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

T0e = FKinBody(M0e,Blist,thetalist');

Tb0 = [1 0 0 0.1662;...
       0 1 0 0;...
       0 0 1 0.0026;...
       0 0 0 1];

l = 0.47/2;
w = 0.3/2;
r = 0.0475;

% Tbe = Tb0*T0e;
% Teb = TransInv(Tbe);
% 
% Ad_Teb = Adjoint(Teb);

% H = 1/r*[-l-w 1 -1;...
%          l+w 1 1;...
%          l+w 1 -1;...
%          -l-w 1 1];
% 
% Jbase = Ad_Teb(:,3:5)*pinv(H);

F = (r/4) * [ -1/(l+w)   1/(l+w)   1/(l+w)  -1/(l+w);
               1          1          1         1;
              -1          1         -1         1 ];

F6 = [ zeros(2,4);
       F;
       zeros(1,4)];
% size(Ad_Teb)
% size(F6)

Jbase = Adjoint(TransInv(T0e)*TransInv(Tb0))*F6;
%Jbase = Ad_Teb * F6;

Je = [Jbase Jarm];

%% calculating the outputs

% twist_d = (1/dt)*MatrixLog6(TransInv(Xd)*Xd_next);
% 
% twist_d = se3ToVec(twist_d);

twist_d = se3ToVec((1/dt)*MatrixLog6(TransInv(Xd)*Xd_next));

% Xe = MatrixLog6(TransInv(X)*Xd);
% 
% Xe = se3ToVec(Xe);

Xe = se3ToVec(MatrixLog6(TransInv(X)*Xd));

Ad = Adjoint(TransInv(X)*Xd);

Xe_int = Xe_int + Xe*dt;

integral = Ki * (Xe_int);

proportional = Kp * Xe;

twist = Adjoint(TransInv(X)*Xd)*twist_d + proportional + integral;

speeds = pinv(Je,1e-2) * twist;

end