%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% David A Smart
% University of Maryland, College Park
% ENPM 661 - Planning for Autonomous Systems
% Project #2 - Path Planning
% 3/3/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Status] = getStatus(NewNode,lQ, res)

    global NodeSet

    % evaluate if new node is in an obstacle
    [Crash] = EvalCrash(NewNode, res);

    % initialize status
    Visited = 0; Goal = 0;
    if ~Crash
        st = max(size(NodeSet.Nodes,3)-4*lQ*res,1)+1;
        if NewNode == NodeSet.Nodes(:,:,1) % solution
            Goal = 1;
        else % not solution
            for i = st:size(NodeSet.Nodes,3) % check *some* previous nodes
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