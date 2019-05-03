%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% David A Smart
% University of Maryland, College Park
% ENPM 661 - Planning for Autonomous Systems
% Project #2 - Path Planning
% 3/3/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Status] = getStatus_DFS(NewNode, res)

    global NodeSet

    % evaluate if new node is in an obstacle
    [Crash] = EvalCrash(NewNode, res);

    % initialize status
    Visited = 0; Goal = 0;
    if ~Crash
        if NewNode == NodeSet.Nodes(:,:,1) % same as Goal node?
            Goal = 1;
        else
            for i = 2:size(NodeSet.Nodes,3) % check all previous nodes
                if NewNode == NodeSet.Nodes(:,:,i) % same as previous node?
                    Visited = 1;
                    break;
                end
            end
        end
    end

    Status(1) = Visited;
    Status(2) = Crash;
    Status(3) = Goal;

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%