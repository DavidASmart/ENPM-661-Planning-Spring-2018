%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% David A Smart
% University of Maryland, College Park
% ENPM 661 - Planning for Autonomous Systems
% Project #2 - Path Planning
% 3/3/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Crash] = EvalCrash(Node)

    % initialize status
    Crash = 0;

    x = Node(1);
    y = Node(2);

    % robot dimenstions
    % H = (15+430)/1000; % m (height = 15 mm ground clearance + 430 mm)
    D = 354/1000; % m (robot diameter = 354 mm)
    r = D/2; % m (robot radius)
    
    % ( conservative )
    % check center of robot
    Crash = EvalCrash_2(x,y);
    % check the edges of the robot
    for th = 0:10:360
        Crash = EvalCrash_2(x+r*cosd(th),y+r*sind(th));
        if Crash
            break;
        end
    end

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
