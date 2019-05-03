%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% David A Smart
% University of Maryland, College Park
% ENPM 661 - Planning for Autonomous Systems
% Project #2 - Path Planning
% 3/3/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Crash] = EvalCrash_2(x,y)

    % check square obstacle
    if x >= 55 && x <= 105 && y >= 67.5 && y <= 112.5
        Crash = 1;

    % check circle obstacle
    elseif (x-180)^2 + (y-120)^2 - 15^2 <= 0
        Crash = 1;

    % check arbitrary obstacle... broken into 4 triangles
    % Triangle #1
    elseif ((14-55)/(145-120))*x - y + (55-(120)*(14-55)/(145-120)) <= 0 && ((51-55)/(158-120))*x - y + (55-(120)*(51-55)/(158-120)) >= 0 && ((51-14)/(158-145))*x - y + (14-145*(51-14)/(158-145)) <= 0
        Crash = 1;
    % Triangle #2
    elseif ((51-14)/(158-145))*x - y + (14-145*(51-14)/(158-145)) >= 0 && y >= 14 && ((14-51)/(168-158))*x - y + (51-158*(14-51)/(168-158)) >= 0
        Crash = 1;
    % Triangle #3
    elseif ((14-51)/(168-158))*x - y + (51-158*(14-51)/(168-158)) <= 0 && y <= 51 && ((51-14)/(188-168))*x - y + (14-168*(51-14)/(188-168)) <= 0
        Crash = 1;
    % Triangle #4
    elseif y >= 51 && ((89-51)/(165-158))*x - y + (51-158*(89-51)/(165-158)) >= 0 && ((51-89)/(188-165))*x - y + (89-165*(51-89)/(188-165)) >= 0
        Crash = 1;
    else
        Crash = 0;
    end

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
