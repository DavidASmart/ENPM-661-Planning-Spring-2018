%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% David A Smart
% University of Maryland, College Park
% ENPM 661 - Planning for Autonomous Systems
% Project #2 - Path Planning
% 3/3/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Crash] = EvalCrash_2(x,y)

    % table 1
    ct1 = [5.525, -3.3750]; % m, center
    st1 = [0.8, 2]; % m, size
    % table 2
    ct2 = [4.325, -3.3750]; % m, center
    st2 = [0.8, 2]; % m, size
    % table 3
    ct3 = [1.1475, 0]; % m, center
    st3 = [2, 3.2]; % m, size 
    % table 4
    ct4 = [-2.6025, 0]; % m, center
    st4 = [2, 3.2]; % m, size 
    % table 5
    ct5 = [-5.35, -4.1]; % m, center
    st5 = [1.6, 1]; % m, size
    % table 6
    ct6 = [-6.95, -0.275]; % m, center
    st6 = [0.8, 2]; % m, size
    % table 7
    ct7 = [-6.95, 1.725]; % m, center
    st7 = [0.8, 2]; % m, size
    
    
    % check area boundaries
    if x >= 7.425 || x <= -7.425 || y >= 4.925 || y <= -4.925
        Crash = 1;
        
    % check table 1
    elseif x >= ct1(1)-st1(1)/2 && x <= ct1(1)+st1(1)/2 && y >= ct1(2)-st1(2)/2 && y <= ct1(2)+st1(2)/2
        Crash = 1;

    % check table 2
    elseif x >= ct2(1)-st2(1)/2 && x <= ct2(1)+st2(1)/2 && y >= ct2(2)-st2(2)/2 && y <= ct2(2)+st2(2)/2
        Crash = 1;
        
    % check table 3
    elseif x >= ct3(1)-st3(1)/2 && x <= ct3(1)+st3(1)/2 && y >= ct3(2)-st3(2)/2 && y <= ct3(2)+st3(2)/2
        Crash = 1;
        
    % check table 4
    elseif x >= ct4(1)-st4(1)/2 && x <= ct4(1)+st4(1)/2 && y >= ct4(2)-st4(2)/2 && y <= ct4(2)+st4(2)/2
        Crash = 1;
    
    % check table 5
    elseif x >= ct5(1)-st5(1)/2 && x <= ct5(1)+st5(1)/2 && y >= ct5(2)-st5(2)/2 && y <= ct5(2)+st5(2)/2
        Crash = 1;
        
    % check table 6
    elseif x >= ct6(1)-st6(1)/2 && x <= ct6(1)+st6(1)/2 && y >= ct6(2)-st6(2)/2 && y <= ct6(2)+st6(2)/2
        Crash = 1;
    
    % check table 7
    elseif x >= ct7(1)-st7(1)/2 && x <= ct7(1)+st7(1)/2 && y >= ct7(2)-st7(2)/2 && y <= ct7(2)+st7(2)/2
        Crash = 1;
        
    else
        Crash = 0;
    end

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 


