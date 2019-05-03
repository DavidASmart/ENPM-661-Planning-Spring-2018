function [x,y] = xy(ds)
% ds = % diffrence in speed between wheels

% robot dimenstions
r = (72/2)/1000; % m (wheel radius)
L = 235/1000; % m (distance between wheels)
% robot limits
umax = 6;%10; % rad/s

% initialize
k = 1;
t(k) = 0; % s
x(k) = 0; % m
y(k) = 0; % m
Theta(k) = 0*(pi/180); % rad

% time increments
dt = 1/20; % s (20 Hz)

if ds > 0
    Ur = umax;
    Ul = umax - ds*umax/100;
elseif ds < 0
    Ur = umax + ds*umax/100;
    Ul = umax;
else 
    Ur = umax;
    Ul = umax;
end

while t(k) <= 0.5 % 2 Hz

    % update...
    k = k + 1;
    
    % time
    t(k) = t(k-1) + dt;

    % theta
    dtheta = (r/L)*(Ur - Ul)*dt;
    Theta(k) = Theta(k-1) + dtheta; % rad

    % x & y
    dx = (r/2)*(Ul + Ur)*cos(Theta(k))*dt;
    x(k) = x(k-1) + dx;
    dy = (r/2)*(Ul + Ur)*sin(Theta(k))*dt;
    y(k) = y(k-1) + dy;
    
end


