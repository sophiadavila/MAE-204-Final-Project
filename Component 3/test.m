% M0e = [1 0 0 0.033;...
%        0 1 0 0;...
%        0 0 1 0.6546;...
%        0 0 0 1];
% 
% Blist = [0 0 1 0 0.033 0;...
%          0 -1 0 -0.5076, 0, 0;...
%          0 -1 0 -0.3526 0 0; ...
%          0 -1 0 -0.2176 0 0; ...
%          0 0 1 0 0 0]';
% 
% thetalist = [0 0 0.2 -1.6 0];
% 
% Jarm = JacobianBody(Blist, thetalist);
% 
% T0e = FKinBody(M0e,Blist,thetalist);
% 
% phi = 0; x = 0; y = 0;
% 
% Tsb = [cos(phi) -sin(phi) 0 x;...
%        sin(phi) cos(phi) 0 y;...
%        0 0 1 0.0963;...
%        0 0 0 1];
% 
% Tb0 = [1 0 0 0.1662;...
%        0 1 0 0;...
%        0 0 1 0.0026;...
%        0 0 0 1];
% 
% l = 0.47/2;
% w = 0.3/2;
% r = 0.0475;
% 
% Tbe = Tb0*T0e;
% Teb = inv(Tbe);
% 
% Ad_Teb = Adjoint(Teb);
% 
% H = 1/r*[-l-w 1 -1;...
%          l+w 1 1;...
%          l+w 1 -1;...
%          -l-w 1 1];
% 
% Jbase = Ad_Teb(:,3:5)*pinv(H);
% 
% Je = [Jbase Jarm];



%% Inputs
Xd = [0 0 1 0.5; 0 1 0 0; -1 0 0 0.5; 0 0 0 1];
Xd_next = [0 0 1 0.6; 0 1 0 0; -1 0 0 0.3; 0 0 0 1];
X = [0.17 0 0.985 0.387; 0 1 0 0; -0.985 0 0.170 0.570; 0 0 0 1];
Kp = eye(6);
Ki = zeros(6);
dt = 0.01;
current_position = [0 0 0 0 0 0.2 -1.6 0];

[twist, speeds, Xe] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, current_position);