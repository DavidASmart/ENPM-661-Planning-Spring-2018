%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% David A Smart
% University of Maryland, College Park
% ENPM 661 - Planning for Autonomous Systems
% Project #2 - Path Planning
% 3/3/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [xytheta,dxdydtheta,vlvr] = Astar_Search (StartNode, GoalNode, res, ress)
    %% Initialize Display and NodeSet Record
    
    % set up display
    InitDisplay(StartNode, GoalNode, res)

    % NodeInfo = [Node #, Parent node #, Cost, Steps]
    GoalNodeInfo = [1, 0, 0, 0 ,0];
    StartNodeInfo = [2, 0, 0, 0, 0];

    % set up the record of nodes first with the GoalNode
    global NodeSet
    NodeSet.Nodes = GoalNode;
    NodeSet.NodesInfo = GoalNodeInfo;
    %global n
    %global np

    Goal = 0;

    % add the starting node to the record of nodes
    AddNode(StartNode, StartNodeInfo);

    % add the starting node to the Que to be expanded
    Que = StartNodeInfo(1);
    
    for i = 1:1:361
        % try moving forward...
        [Que, Goal] = Move(0, StartNode, StartNodeInfo, Que, res); % move left
        if length(Que) ~= 1
            break;
        else
            % if it does not work, rotate ...
            StartNode(3) = StartNode(3) + 1;
        end
    end
    
    % reinitialize search
    % deleted old node & path
    %delete(np(:))
    %delete(n(3:end))
    Que = StartNodeInfo(1);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Expand Nodes

    while ~Goal % until the space has been explored

        % initialize index for minimum cost to first in Que
        indexMC = min(find(NodeSet.NodesInfo(:,1,:) == Que(1)));
        % search nodes in Que for lower cost
        for i = 2:length(Que)
            index = min(find(NodeSet.NodesInfo(:,1,:) == Que(i)));
            if NodeSet.NodesInfo(:,3,index) < NodeSet.NodesInfo(:,3,indexMC)
                indexMC = index;
            end
        end
        % take the node in the Que with the lowest cost
        CurrentNode = NodeSet.Nodes(:,:,indexMC);
        CurrentNodeInfo = NodeSet.NodesInfo(:,:,indexMC);
        
        % expand node
        for ds = -100:ress:100  % increments from -100 to 100 percent
            [Que, Goal] = Move(ds,CurrentNode, CurrentNodeInfo, Que, res); % move
        end

%         if CurrentNodeInfo(1,1,1) > 2
%             % establish block
%             th = 0:15:360;
%             bX = (res/4)*cosd(th);
%             bY = (res/4)*sind(th);
%             % update color of node to indicate that is has been expanded
%             delete(n(CurrentNodeInfo(1)))
%             n(CurrentNodeInfo(1)) = patch(CurrentNode(1)+bX,CurrentNode(2)+bY,'y');
%             axis equal
%             pause(0.0001);
%         end

        % update the Que
        index_old = find(Que(:) == CurrentNodeInfo(1));
        Que(index_old) = [];

    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Generate Optimal Path

    % find the terminal node
%     for i = 3:size(NodeSet.Nodes,3) % check all node beside the start
%         if  sqrt((NodeSet.Nodes(1,1,i) - NodeSet.Nodes(1,1,1))^2 + (NodeSet.Nodes(1,2,i) - NodeSet.Nodes(1,2,1))^2) < res
%             TerminalNodeInfo = NodeSet.NodesInfo(:,:,i);
%             TerminalNode = NodeSet.Nodes(:,:,i);            
%             break;
%         end
%     end

    % stoping when goal found, so last one
    TerminalNodeInfo = NodeSet.NodesInfo(:,:,end);
    TerminalNode = NodeSet.Nodes(:,:,end);
    

    % lets backtrack through the parent nodes to find the full path
    steps = TerminalNodeInfo(4); % number of steps taken to get here
    Path(steps+1,:) = TerminalNode;
    PathInfo(steps+1,:) = TerminalNodeInfo; % start at the termpinal node
    
    % establish block
    th = 0:15:360;
    D = 354/1000; % m (robot diameter = 354 mm)
    r = D/2; % m (robot radius)
    bX = r*cosd(th);
    bY = r*sind(th);
    
    while steps > 0 % until we have arrived back at the start node
        pN = PathInfo(steps+1,2); % get the parent node number
        PathInfo(steps,:) = NodeSet.NodesInfo(:,:,pN); % save the parent node's info
        Path(steps,:) = NodeSet.Nodes(:,:,pN); % save the path
        patch(Path(steps,1)+bX*2,Path(steps,2)+bY*2,'g');
        pause(0.0001);
        steps = steps - 1; % move on to that parent node
    end
    
    % split node information up
    xytheta = Path(:,1:3);
    dxdydtheta = Path(:,4:6);
    vlvr = Path(:,7:8);
    
    % save figure
    saveas(gcf,strcat('Path.jpg'));
    
    
    
end
