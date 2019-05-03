%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% David A Smart
% University of Maryland, College Park
% ENPM 661 - Planning for Autonomous Systems
% Project #2 - Path Planning
% 3/3/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [NewNodeInfo] = getNewNodeInfo(NewNode, CurrentNodeInfo, s)
    
    global NodeSet    
    
    % new Node # = number of existing nodes + 1
    NewNodeNo = size(NodeSet.NodesInfo,3)+1;
    
    % new node's Parent node # = current node
    ParentNodeNo = CurrentNodeInfo(1);
    
    % new node's steps to come = current node steps to come + 1 ...
    Steps = CurrentNodeInfo(4)+1;
    % new node's cost to come = current node cost to come + distance traveled ...
    Cost2Come = CurrentNodeInfo(5) + s;
    
    % ... sumed with the distance to the goal is the total cost
    dxG = abs(NodeSet.Nodes(1,1,1)-NewNode(1));
    dyG = abs(NodeSet.Nodes(1,2,1)-NewNode(2));
    Cost = Cost2Come + sqrt(dxG^2+dyG^2)*1;
    
    % NodeInfo = [Node #, Parent node #, Cost]
    NewNodeInfo = [NewNodeNo, ParentNodeNo, Cost, Steps, Cost2Come];
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%