%% method
% satisfy differential constaints while creating the path
% convert path to trajectory
% note: always controllable
% pruning: check nodes for nearby nodes (within some radius or square 'node') and choose one with lowest cost

%% output.txt = [ul,ur] @ 2 Hz

%% alternative for diferential equations
% u_w = ((ul+ur)/2); % "translation"
% u_psi = (ur-ur); % "rotation"

% x_dot = r*u_w*cos(theta);
% y_dot = r*u_w*sin(theta);
% theta_dot = (r/L)*u_psi;

%% Relative Velocities

% world frame
% dtheta = (r/L)*(ur - ul);
% dx(k) = (r/2)*(ul + ur)*cos(theta(k));
% dy(k) = (r/2)*(ul + ur)*sin(theta(k));

% robot frame
% vl = ul*r; % m/s
% vr = ur*r; % m/s
% % linear velocity of robot center
%********************************* v = (vr+vl)/2; % m/s
% v = sqrt((dx*2)^2+(dy*2)^2)/2

%********************************** w = (ur-ur)
% w = dtheta*(L/r)