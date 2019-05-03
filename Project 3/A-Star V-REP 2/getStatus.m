%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% David A Smart
% University of Maryland, College Park
% ENPM 661 - Planning for Autonomous Systems
% Project #2 - Path Planning
% 3/3/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Visited, Crash, Goal] = getStatus(Node,res)

    global NodeSet

    % evaluate if new node is in an obstacle
    [Crash] = EvalCrash(Node);

    % initialize status
    Visited = 0; Goal = 0;
    
    D = 354/1000; % m (robot diameter = 354 mm)
    if sqrt((Node(1) - NodeSet.Nodes(1,1,1))^2 + (Node(2) - NodeSet.Nodes(1,2,1))^2) < D % same as Goal node?
        Goal = 1;
    end
    if ~Crash
        for i = 3:size(NodeSet.Nodes,3) % check all previous nodes
            d = sqrt((Node(1) - NodeSet.Nodes(:,1,i))^2 + (Node(2) - NodeSet.Nodes(:,2,i))^2);
            if d < res % same as previous node?
                % fprintf('\n d = %3.3f',d);
                % fprintf('\n visited = %i',i);
                if Visited(1) ~= 0
                    Visited(end+1) = NodeSet.Nodes(1,1,i); % which node is it equivilent too?
                else
                    Visited = NodeSet.Nodes(1,1,i); % which node is it equivilent too?
                end
            end
        end
    else
        % fprintf('\n crash!');
    end

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%