%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% David A Smart
% University of Maryland, College Park
% ENPM 661 - Planning for Autonomous Systems
% Project #2 - Path Planning
% 3/3/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Que, GoalReached] = Move(dx, dy, CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res)

    % move
    NewNode(1) = CurrentNode(1)+dx*res;       % x
    NewNode(2) = CurrentNode(2)+dy*res;       % y
    
    % Status?
    if ~METHOD(1)
        [Status] = getStatus_DFS(NewNode, res);
    else
        [Status] = getStatus(NewNode,length(Que), res);
    end

    % getNewNodeInfo   
    if METHOD(3)
        [NewNodeInfo] = getNewNodeInfo_Astar(NewNode, CurrentNodeInfo, dx, dy, weight);
    else
        [NewNodeInfo] = getNewNodeInfo(CurrentNodeInfo);
    end
    
    % Add NewNode to record and Que
    if Status(1) == 0 && Status(2) == 0     % if not already visited
        AddNode(NewNode, NewNodeInfo);      % add to record
        % establish block
        bX = [-0.5,-0.5,0.5,0.5,-0.5]*res;
        bY = [-0.5,0.5,0.5,-0.5,-0.5]*res;
        % plot node
        patch(NewNode(1)+bX,NewNode(2)+bY,'g');
        pause(0.0001);
        if Status(3) == 0                   % if not a goal node
            Que(end+1) = NewNodeInfo(1);    % add to the Que to be expanded
        end
    end

    GoalReached = Status(3);
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
