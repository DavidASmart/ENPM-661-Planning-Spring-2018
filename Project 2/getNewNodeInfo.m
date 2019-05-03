%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% David A Smart
% University of Maryland, College Park
% ENPM 661 - Planning for Autonomous Systems
% Project #2 - Path Planning
% 3/3/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [NewNodeInfo] = getNewNodeInfo_BFS(CurrentNodeInfo)
    
    global NodeSet    
    
    % new Node # = number of existing nodes + 1
    NewNodeNo = size(NodeSet.Nodes,3)+1;
    
    % new node's Parent node # = current node
    ParentNodeNo = CurrentNodeInfo(1);
    
    % new node's cost to come = current node cost to come + 1 
    Cost2Come = CurrentNodeInfo(3)+1;
    
    % NodeInfo = [Node #, Parent node #, CostToCome]
    NewNodeInfo = [NewNodeNo, ParentNodeNo, Cost2Come];
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%