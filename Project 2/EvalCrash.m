%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% David A Smart
% University of Maryland, College Park
% ENPM 661 - Planning for Autonomous Systems
% Project #2 - Path Planning
% 3/3/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Crash] = EvalCrash(NewNode, res)

    % initialize status
    Crash = 0;

    x = NewNode(1);
    y = NewNode(2);

% ( true point-robot )
%     % just check the center of the "block"
%     C = EvalCrash_2(x,y);
% 
%     if C
%         Crash = 1;
%     end

% ( conservative )
    % check the edges of the "block"
    CLU = EvalCrash_2(x-0.5*res,y+0.5*res);
    CRU = EvalCrash_2(x+0.5*res,y+0.5*res);
    CLD = EvalCrash_2(x-0.5*res,y-0.5*res);
    CRD = EvalCrash_2(x+0.5*res,y-0.5*res);

    if CLU || CRU || CRD || CLD
        Crash = 1;
    end

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% y = m*x + y0 --> ax + by + c = 0
% a = m, b = -1, c = y0


% Triangle #1
%   Line #1                 %   Line #2                 %   Line #3 
%       "left" (>)          %       "top" (<)           %       "right"(>)
%       x1,y1 = (120,55)	%       x1,y1 = (120,55)    %       x1,y1 = (145,14)
%       x2,y2 = (145,14)    %       x2,y2 = (158,51)    %       x2,y2 = (158,51)
%       m ~ -1.64           %       m ~ -0.105          %       m ~ 2.85
%       y0 ~ 251.8          %       y0 ~ 67.63          %       y0 ~ -398.7

% Triangle #2
%   Line #3                 %   Line #4                 %   Line #5
%       "left"(<)           %       "bottom" (>)        %       "right" (<)
%       x1,y1 = (145,14)    %       [y = 14]            %       x1,y1 = (158,51)
%       x1,y1 = (158,51)                                %       x2,y2 = (168,14)
%       m ~ 2.85                                        %       m = -3.7
%       y0 ~ -398.7                                     %       y0 = 635.6

% Triangle #3
%   Line #5                 %   Line #6                 %   Line #7
%       "left" (>)          %       "top" (<)           %       "right" (>)
%       x1,y1 = (158,51)    %       [y = 51]            %       x1,y1 = (168,14)
%       x2,y2 = (168,14)                                %       x2,y2 = (188,51)
%       m = -3.7                                        %       m = 1.85
%       y0 = 635.6                                      %       y0 = -296.8

% Triangle #4
%   Line #6                 %   Line #8                 %   Line #9
%       "bottom" (>)        %       "left" (<)          %       "right" (<)
%       y = 51              %       x1,y1 = (158,51)    %       x1,y1 = (165,89)
                            %       x2,y2 = (165,89)    %       x2,y2 = (188,51)
                            %       m =  ~ 5.43         %       m ~ -1.65
                            %       y0 = -806.7         %       y0 ~ 361.61