%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% David A Smart
% University of Maryland, College Park
% ENPM 661 - Planning for Autonomous Systems
% Project #2 - Path Planning
% 3/3/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear
clc

%% ***select search method****************************
% METHOD = [BFS, DFS, A*]
METHOD = [1,0,0]; % ONLY PICK ONE!

% use the direction to the goal to order expansion?
smart = 1;

% if using "weighted" A*
weight = 1;

% select resolution
res = 4;

%% ***select start and goal****************************
% % EASY
% StartNode = res*floor([180+res,30-res]/res);
% GoalNode = res*floor([60-res,120+res]/res);
% 
% DIFFICULT
StartNode = res*floor([60-res,120+res]/res);
GoalNode = res*floor([180+res,30-res]/res);

% % RANDOM
% % until both start and goal are initialized in free space
% inObs = 1; dxy = 0;
% while inObs || dxy < 50
% 
%     % generate random start and end points
%     StartNode = res*floor([randi(250),randi(150)]/res);
%     GoalNode = res*floor([randi(250),randi(150)]/res);
% 
%     dx = abs(StartNode(1)-GoalNode(1));
%     dy = abs(StartNode(2)-GoalNode(2));
%     dxy = sqrt(dx^2+dy^2);
% 
%     % check if they are in an obstical
%     Crash_Start = EvalCrash(StartNode,res);
%     Crash_Goal = EvalCrash(GoalNode,res);
%     if ~Crash_Start && ~Crash_Goal
%         inObs = 0;
%     end
% 
% end

clear Crash_Start Crash_Goal inObs dx dy dxy

%% ***Search******************************************
if METHOD(1) % search using BFS
    if smart
        Path = Breadth_First_Search_2 (StartNode, GoalNode, METHOD, res);
        save(strcat('Path_BFS_smart_',num2str(res),'.mat'),'Path');
        saveas(gcf,strcat('Path_BFS_smart_',num2str(res),'.jpg'))
    else
        Path = Breadth_First_Search (StartNode, GoalNode, METHOD, res);
        save(strcat('Path_BFS_',num2str(res),'.mat'),'Path');
        saveas(gcf,strcat('Path_BFS_',num2str(res),'.jpg'))
    end


elseif METHOD(2) % search using DFS
    if smart
        Path = Depth_First_Search_2 (StartNode, GoalNode, METHOD, res);
        save(strcat('Path_DFS_smart_',num2str(res),'.mat'),'Path');
        saveas(gcf,strcat('Path_DFS_smart_',num2str(res),'.jpg'))
    else
        Path = Depth_First_Search (StartNode, GoalNode, METHOD, res);
        save(strcat('Path_DFS_',num2str(res),'.mat'),'Path');
        saveas(gcf,strcat('Path_DFS_',num2str(res),'.jpg'))
    end


elseif METHOD(3) % search using Astar
    if smart
        Path = Astar_Search_2 (StartNode, GoalNode, METHOD, weight,res);
        
        save(strcat('Path_Astar_smart_',num2str(weight),'_',num2str(res),'.mat'),'Path');
        saveas(gcf,strcat('Path_Astar_smart_',num2str(weight),'_',num2str(res),'.jpg'))
    else
        Path = Astar_Search (StartNode, GoalNode, METHOD, weight);
        save(strcat('Path_Astar_',num2str(weight),'_',num2str(res),'.mat'),'Path');
        saveas(gcf,strcat('Path_Astar_',num2str(weight),'_',num2str(res),'.jpg'))
    end
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%