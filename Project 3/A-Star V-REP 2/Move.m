%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% David A Smart
% University of Maryland, College Park
% ENPM 661 - Planning for Autonomous Systems
% Project #2 - Path Planning
% 3/3/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Que, Goal] = Move(ds, CurrentNode, CurrentNodeInfo, Que, res)
% td - deg

    Goal = 0;

    % robot dimenstions
    r = (72/2)/1000; % m (wheel radius)
    L = 235/1000; % m (distance between wheels)
    % robot limits
    umax = 6;%10; % rad/s

    % initialize
    k = 1;
    t(k) = 0; % sec
    x(k) = CurrentNode(1); % m
    y(k) = CurrentNode(2); % m
    theta(k) = CurrentNode(3)*(pi/180); % rad
    s = 0; % distance traveled
    pathcrash = 0;
    
    % time increments
    dt = 1/20; % s (20 Hz)
    
    % wheel velocities
    if ds > 0
        ur = umax;
        ul = umax - ds*umax/100;
    elseif ds < 0
        ur = umax + ds*umax/100;
        ul = umax;
    else 
        ur = umax;
        ul = umax;
    end   
    
    while t(k) <= 0.5 % 2 Hz

        % update...
        k = k + 1;

        % time
        t(k) = t(k-1) + dt;

        % theta
        dtheta = (r/L)*(ur - ul)*dt;
        theta(k) = theta(k-1) + dtheta; % rad

        % x & y
        dx(k) = (r/2)*(ul + ur)*cos(theta(k))*dt;
        x(k) = x(k-1) + dx(k);
        dy(k) = (r/2)*(ul + ur)*sin(theta(k))*dt;
        y(k) = y(k-1) + dy(k);
        
        % preiodicaly along the path
        if mod(k,5) == 0 % 4 Hz?
            pathcrash = EvalCrash([x(k),y(k)]); % check for collision
            if pathcrash
                break;
            end
        end
        
        % total distance traveled
        s = s + sqrt(dx(k)^2+dy(k)^2);

    end       
    
    if ~pathcrash % don't bother doing anything else if the robot crashes getting here
        % make new node
        NewNode = [x(k),y(k),theta(k)*(180/pi),mean(dx),mean(dy),dtheta*(180/pi),ul,ur];
        
        % Status?
        [Visited, Crash, Goal] = getStatus(NewNode, res);

        % getNewNodeInfo
        [NewNodeInfo] = getNewNodeInfo(NewNode, CurrentNodeInfo, s);

        % compare new node against record ...
        % ... then update the record, Que, and display
        [Que] = UpdateQueAndDisplay(NewNode, NewNodeInfo, Que, Visited, Crash, Goal, x, y, res);
    end
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
