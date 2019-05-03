%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% David A Smart
% University of Maryland, College Park
% ENPM 661 - Planning for Autonomous Systems
% Project #2 - Path Planning
% 3/3/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [NewNodeInfo] = getNewNodeInfo_Astar(NewNode, CurrentNodeInfo, dx, dy, weight)
    
    global NodeSet    
    
    % new Node # = number of existing nodes + 1
    NewNodeNo = size(NodeSet.Nodes,3)+1;
    
    % new node's Parent node # = current node
    ParentNodeNo = CurrentNodeInfo(1);
    
    % new node's steps to come = current node steps to come + 1 ...
    Steps = CurrentNodeInfo(4)+1;
    Cost2Come = CurrentNodeInfo(5) + sqrt(dx^2+dy^2);
    % ... sumed with the distance to the goal
    dxG = abs(NodeSet.Nodes(1,1,1)-NewNode(1));
    dyG = abs(NodeSet.Nodes(1,2,1)-NewNode(2));
    Cost =  Cost2Come + sqrt(dxG^2+dyG^2)*weight;
    
    % NodeInfo = [Node #, Parent node #, Cost, # Steps, Cost2Come]
    NewNodeInfo = [NewNodeNo, ParentNodeNo, Cost, Steps, Cost2Come];
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%