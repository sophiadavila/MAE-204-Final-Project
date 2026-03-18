function [V, speeds, Xerr] = FeedbackControl2(X, Xd, Xd_next, Kp, Ki, dt, thetalist)

persistent Xerr_integral

if isempty(Xerr_integral)
    Xerr_integral = zeros(6,1);
end

% Robot constants
M0e = [1 0 0 0.033;
       0 1 0 0;
       0 0 1 0.6546;
       0 0 0 1];

Blist = [0 0 1 0 0.033 0;
         0 -1 0 -0.5076 0 0;
         0 -1 0 -0.3526 0 0;
         0 -1 0 -0.2176 0 0;
         0 0 1 0 0 0]';

Tb0 = [1 0 0 0.1662;
       0 1 0 0;
       0 0 1 0.0026;
       0 0 0 1];

r = 0.0475;
l = 0.47/2;
w = 0.3/2;

F = (r/4)*[-1/(l+w)  1/(l+w)  1/(l+w) -1/(l+w);
            1         1         1         1;
           -1         1        -1         1];

% --- Error twist ---
Xerr = se3ToVec(MatrixLog6(TransInv(X)*Xd));

% --- Desired twist ---
Vd = se3ToVec(MatrixLog6(TransInv(Xd)*Xd_next))/dt;

% --- Feedforward twist in body frame ---
Vd = Adjoint(TransInv(X)*Xd)*Vd;

% --- Integral error ---
Xerr_integral = Xerr_integral + Xerr*dt;

% --- Commanded twist ---
V = Vd + Kp*Xerr + Ki*Xerr_integral;

% --- Arm Jacobian ---
Jarm = JacobianBody(Blist,thetalist);

% --- Current arm pose ---
T0e = FKinBody(M0e,Blist,thetalist);

% --- Base Jacobian ---
Jbase = Adjoint(TransInv(T0e)*TransInv(Tb0))*[zeros(3,4);F];

% --- Mobile manipulator Jacobian ---
Je = [Jbase Jarm];

% --- Wheel + joint speeds ---
speeds = pinv(Je)*V;

end