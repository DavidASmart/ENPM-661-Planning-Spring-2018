%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% David A Smart
% University of Maryland, College Park
% ENPM 661 - Planning for Autonomous Systems
% Project #2 - Path Planning
% 3/3/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Path] = Depth_First_Search_2 (StartNode, GoalNode, METHOD, res)

    %% Initialize Display and NodeSet Record

    % set up display
    InitDisplay(StartNode, GoalNode, res)

    % NodeInfo = [Node #, Parent node #, CostToCome]
    GoalNodeInfo = [1, 0, 0];
    StartNodeInfo = [2, 0, 0];

    % set up the record of nodes first with the GoalNode
    global NodeSet
    NodeSet.Nodes = GoalNode;
    NodeSet.NodesInfo = GoalNodeInfo;

    % add the starting node to the record of nodes
    AddNode(StartNode, StartNodeInfo);

    % add the starting node to the Que to be expanded
    Que(1) = StartNodeInfo(1);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Expand Nodes... using knowledge of goal direction!
    GoalReached = 0; weight = 1;
    while ~GoalReached % until the goal is reached

        % take the node from the the front of the Que
        index = Que(end);
        CurrentNode = NodeSet.Nodes(:,:,index);
        CurrentNodeInfo = NodeSet.NodesInfo(:,:,index);

        % determine direction from current node to goal
        dx = GoalNode(1)-CurrentNode(1);
        dy = GoalNode(2)-CurrentNode(2);

        % depending on the direction,try to expand in a certain order
        if dx > 0 && dy > 0 % if to up-right
            if dx > dy % if more right than up
                 if CurrentNode(1) > 1 && CurrentNode(2) > 1 % if not in the bottom-left corner
                    [Que, GoalReached] = Move(-1,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down & left
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) > 1  % if not on the left edge
                    [Que, GoalReached] = Move(-1,0,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move left
                    if GoalReached
                        break
                    end
                end               
                if CurrentNode(2) > 1  % if not on the bottom edge
                    [Que, GoalReached] = Move(0,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) > 1 && CurrentNode(2) < 150 % if not in the top-left corner
                    [Que, GoalReached] = Move(-1,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up & left
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) < 250 && CurrentNode(2) > 1 % if not in the bottom-right corner
                    [Que, GoalReached] = Move(1,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down & right
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(2) < 150  % if not on the top edge
                    [Que, GoalReached] = Move(0,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) < 250 && CurrentNode(2) < 150 % if not in the top-right corner
                    [Que, GoalReached] = Move(1,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up & righ
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) < 250  % if not on the right edge
                    [Que, GoalReached] = Move(1,0,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move right
                    if GoalReached
                        break
                    end
                end

            else % (dy > dx) % if more up than right
                if CurrentNode(1) > 1 && CurrentNode(2) > 1 % if not in the bottom-left corner
                    [Que, GoalReached] = Move(-1,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down & left
                    if GoalReached
                        break
                    end
                end               
                if CurrentNode(2) > 1  % if not on the bottom edge
                    [Que, GoalReached] = Move(0,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) > 1  % if not on the left edge
                    [Que, GoalReached] = Move(-1,0,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move left
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) < 250 && CurrentNode(2) > 1 % if not in the bottom-right corner
                    [Que, GoalReached] = Move(1,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down & right
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) > 1 && CurrentNode(2) < 150 % if not in the top-left corner
                    [Que, GoalReached] = Move(-1,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up & left
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) < 250  % if not on the right edge
                    [Que, GoalReached] = Move(1,0,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move right
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) < 250 && CurrentNode(2) < 150 % if not in the top-right corner
                    [Que, GoalReached] = Move(1,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up & righ
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(2) < 150  % if not on the top edge
                    [Que, GoalReached] = Move(0,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up
                    if GoalReached
                        break
                    end
                end
            end


        elseif dx < 0 && dy > 0 % if to up-left
            if abs(dx) > dy % if more left than up
                if CurrentNode(1) < 250 && CurrentNode(2) > 1 % if not in the bottom-right corner
                    [Que, GoalReached] = Move(1,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down & right
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) < 250  % if not on the right edge
                    [Que, GoalReached] = Move(1,0,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move right
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(2) > 1  % if not on the bottom edge
                    [Que, GoalReached] = Move(0,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) < 250 && CurrentNode(2) < 150 % if not in the top-right corner
                    [Que, GoalReached] = Move(1,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up & righ
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) > 1 && CurrentNode(2) > 1 % if not in the bottom-left corner
                    [Que, GoalReached] = Move(-1,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down & left
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(2) < 150  % if not on the top edge
                    [Que, GoalReached] = Move(0,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) > 1 && CurrentNode(2) < 150 % if not in the top-left corner
                    [Que, GoalReached] = Move(-1,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up & left
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) > 1  % if not on the left edge
                    [Que, GoalReached] = Move(-1,0,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move left
                    if GoalReached
                        break
                    end
                end

            else % (dy > abs(dx)) % if more up than left
                if CurrentNode(1) < 250 && CurrentNode(2) > 1 % if not in the bottom-right corner
                    [Que, GoalReached] = Move(1,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down & right
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(2) > 1  % if not on the bottom edge
                    [Que, GoalReached] = Move(0,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) < 250  % if not on the right edge
                    [Que, GoalReached] = Move(1,0,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move right
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) > 1 && CurrentNode(2) > 1 % if not in the bottom-left corner
                    [Que, GoalReached] = Move(-1,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down & left
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) < 250 && CurrentNode(2) < 150 % if not in the top-right corner
                    [Que, GoalReached] = Move(1,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up & righ
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) > 1  % if not on the left edge
                    [Que, GoalReached] = Move(-1,0,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move left
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) > 1 && CurrentNode(2) < 150 % if not in the top-left corner
                    [Que, GoalReached] = Move(-1,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up & left
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(2) < 150  % if not on the top edge
                    [Que, GoalReached] = Move(0,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up
                    if GoalReached
                        break
                    end
                end
            end


        elseif dx > 0 && dy < 0 % if to down-right
            if dx > abs(dy) % if more right than down
                if CurrentNode(1) > 1 && CurrentNode(2) < 150 % if not in the top-left corner
                    [Que, GoalReached] = Move(-1,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up & left
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) > 1  % if not on the left edge
                    [Que, GoalReached] = Move(-1,0,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move left
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(2) < 150  % if not on the top edge
                    [Que, GoalReached] = Move(0,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) > 1 && CurrentNode(2) > 1 % if not in the bottom-left corner
                    [Que, GoalReached] = Move(-1,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down & left
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) < 250 && CurrentNode(2) < 150 % if not in the top-right corner
                    [Que, GoalReached] = Move(1,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up & righ
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(2) > 1  % if not on the bottom edge
                    [Que, GoalReached] = Move(0,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) < 250 && CurrentNode(2) > 1 % if not in the bottom-right corner
                    [Que, GoalReached] = Move(1,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down & right
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) < 250  % if not on the right edge
                    [Que, GoalReached] = Move(1,0,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move right
                    if GoalReached
                        break
                    end
                end

            else % (abs(dy) > dx) % if more down than right
                if CurrentNode(1) > 1 && CurrentNode(2) < 150 % if not in the top-left corner
                    [Que, GoalReached] = Move(-1,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up & left
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(2) < 150  % if not on the top edge
                    [Que, GoalReached] = Move(0,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) > 1  % if not on the left edge
                    [Que, GoalReached] = Move(-1,0,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move left
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) < 250 && CurrentNode(2) < 150 % if not in the top-right corner
                    [Que, GoalReached] = Move(1,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up & righ
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) > 1 && CurrentNode(2) > 1 % if not in the bottom-left corner
                    [Que, GoalReached] = Move(-1,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down & left
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) < 250  % if not on the right edge
                    [Que, GoalReached] = Move(1,0,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move right
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) < 250 && CurrentNode(2) > 1 % if not in the bottom-right corner
                    [Que, GoalReached] = Move(1,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down & right
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(2) > 1  % if not on the bottom edge
                    [Que, GoalReached] = Move(0,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down
                    if GoalReached
                        break
                    end
                end
            end


        elseif dx < 0 && dy < 0 % if to down-left
            if abs(dx) > abs(dy) % if more left than down
                if CurrentNode(1) < 250 && CurrentNode(2) < 150 % if not in the top-right corner
                    [Que, GoalReached] = Move(1,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up & righ
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) < 250  % if not on the right edge
                    [Que, GoalReached] = Move(1,0,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move right
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(2) < 150  % if not on the top edge
                    [Que, GoalReached] = Move(0,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) < 250 && CurrentNode(2) > 1 % if not in the bottom-right corner
                    [Que, GoalReached] = Move(1,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down & right
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) > 1 && CurrentNode(2) < 150 % if not in the top-left corner
                    [Que, GoalReached] = Move(-1,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up & left
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(2) > 1  % if not on the bottom edge
                    [Que, GoalReached] = Move(0,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) > 1 && CurrentNode(2) > 1 % if not in the bottom-left corner
                    [Que, GoalReached] = Move(-1,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down & left
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) > 1  % if not on the left edge
                    [Que, GoalReached] = Move(-1,0,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move left
                    if GoalReached
                        break
                    end
                end

            else % (abs(dy) > abs(dx)) % if more down than left
                if CurrentNode(1) < 250 && CurrentNode(2) < 150 % if not in the top-right corner
                    [Que, GoalReached] = Move(1,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up & righ
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(2) < 150  % if not on the top edge
                    [Que, GoalReached] = Move(0,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) < 250  % if not on the right edge
                    [Que, GoalReached] = Move(1,0,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move right
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) > 1 && CurrentNode(2) < 150 % if not in the top-left corner
                    [Que, GoalReached] = Move(-1,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up & left
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) < 250 && CurrentNode(2) > 1 % if not in the bottom-right corner
                    [Que, GoalReached] = Move(1,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down & right
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) > 1  % if not on the left edge
                    [Que, GoalReached] = Move(-1,0,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move left
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(1) > 1 && CurrentNode(2) > 1 % if not in the bottom-left corner
                    [Que, GoalReached] = Move(-1,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down & left
                    if GoalReached
                        break
                    end
                end
                if CurrentNode(2) > 1  % if not on the bottom edge
                    [Que, GoalReached] = Move(0,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down
                    if GoalReached
                        break
                    end
                end
            end


        elseif dx == 0 && dy > 0 % if up
            if CurrentNode(2) > 1  % if not on the bottom edge
                [Que, GoalReached] = Move(0,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down
                if GoalReached
                    break
                end
            end
            if CurrentNode(1) > 1 && CurrentNode(2) > 1 % if not in the bottom-left corner
                [Que, GoalReached] = Move(-1,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down & left
                if GoalReached
                    break
                end
            end
            if CurrentNode(1) < 250 && CurrentNode(2) > 1 % if not in the bottom-right corner
                [Que, GoalReached] = Move(1,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down & right
                if GoalReached
                    break
                end
            end
            if CurrentNode(1) > 1  % if not on the left edge
                [Que, GoalReached] = Move(-1,0,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move left
                if GoalReached
                    break
                end
            end
            if CurrentNode(1) < 250  % if not on the right edge
                [Que, GoalReached] = Move(1,0,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move right
                if GoalReached
                    break
                end
            end
            if CurrentNode(1) > 1 && CurrentNode(2) < 150 % if not in the top-left corner
                [Que, GoalReached] = Move(-1,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up & left
                if GoalReached
                    break
                end
            end
            if CurrentNode(1) < 250 && CurrentNode(2) < 150 % if not in the top-right corner
                [Que, GoalReached] = Move(1,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up & righ
                if GoalReached
                    break
                end
            end
            if CurrentNode(2) < 150  % if not on the top edge
                [Que, GoalReached] = Move(0,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up
                if GoalReached
                    break
                end
            end


        elseif dx == 0 && dy < 0 % if down
            if CurrentNode(2) < 150  % if not on the top edge
                [Que, GoalReached] = Move(0,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up
                if GoalReached
                    break
                end
            end
            if CurrentNode(1) < 250 && CurrentNode(2) < 150 % if not in the top-right corner
                [Que, GoalReached] = Move(1,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up & right
                if GoalReached
                    break
                end
            end
            if CurrentNode(1) > 1 && CurrentNode(2) < 150 % if not in the top-left corner
                [Que, GoalReached] = Move(-1,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up & left
                if GoalReached
                    break
                end
            end
            if CurrentNode(1) < 250  % if not on the right edge
                [Que, GoalReached] = Move(1,0,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move right
                if GoalReached
                    break
                end
            end
            if CurrentNode(1) > 1  % if not on the left edge
                [Que, GoalReached] = Move(-1,0,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move left
                if GoalReached
                    break
                end
            end
            if CurrentNode(1) < 250 && CurrentNode(2) > 1 % if not in the bottom-right corner
                [Que, GoalReached] = Move(1,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down & right
                if GoalReached
                    break
                end
            end
            if CurrentNode(1) > 1 && CurrentNode(2) > 1 % if not in the bottom-left corner
                [Que, GoalReached] = Move(-1,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down & left
                if GoalReached
                    break
                end
            end
            if CurrentNode(2) > 1  % if not on the bottom edge
                [Que, GoalReached] = Move(0,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down
                if GoalReached
                    break
                end
            end


        elseif dx > 0 && dy == 0 % if right
            if CurrentNode(1) > 1  % if not on the left edge
                [Que, GoalReached] = Move(-1,0,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move left
                if GoalReached
                    break
                end
            end
            if CurrentNode(1) > 1 && CurrentNode(2) > 1 % if not in the bottom-left corner
                [Que, GoalReached] = Move(-1,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down & left
                if GoalReached
                    break
                end
            end
            if CurrentNode(1) > 1 && CurrentNode(2) < 150 % if not in the top-left corner
                [Que, GoalReached] = Move(-1,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up & left
                if GoalReached
                    break
                end
            end
            if CurrentNode(2) > 1  % if not on the bottom edge
                [Que, GoalReached] = Move(0,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down
                if GoalReached
                    break
                end
            end
            if CurrentNode(2) < 150  % if not on the top edge
                [Que, GoalReached] = Move(0,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up
                if GoalReached
                    break
                end
            end
            if CurrentNode(1) < 250 && CurrentNode(2) > 1 % if not in the bottom-right corner
                [Que, GoalReached] = Move(1,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down & right
                if GoalReached
                    break
                end
            end
            if CurrentNode(1) < 250 && CurrentNode(2) < 150 % if not in the top-right corner
                [Que, GoalReached] = Move(1,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up & right
                if GoalReached
                    break
                end
            end
            if CurrentNode(1) < 250  % if not on the right edge
                [Que, GoalReached] = Move(1,0,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move right
                if GoalReached
                    break
                end
            end


        elseif dx < 0 && dy == 0 % if left
            if CurrentNode(1) < 250  % if not on the right edge
                [Que, GoalReached] = Move(1,0,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move right
                if GoalReached
                    break
                end
            end
            if CurrentNode(1) < 250 && CurrentNode(2) < 150 % if not in the top-right corner
                [Que, GoalReached] = Move(1,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up & righ
                if GoalReached
                    break
                end
            end
            if CurrentNode(1) < 250 && CurrentNode(2) > 1 % if not in the bottom-right corner
                [Que, GoalReached] = Move(1,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down & right
                if GoalReached
                    break
                end
            end
            if CurrentNode(2) < 150  % if not on the top edge
                [Que, GoalReached] = Move(0,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up
                if GoalReached
                    break
                end
            end
            if CurrentNode(2) > 1  % if not on the bottom edge
                [Que, GoalReached] = Move(0,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down
                if GoalReached
                    break
                end
            end
            if CurrentNode(1) > 1 && CurrentNode(2) < 150 % if not in the top-left corner
                [Que, GoalReached] = Move(-1,1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move up & left
                if GoalReached
                    break
                end
            end
            if CurrentNode(1) > 1 && CurrentNode(2) > 1 % if not in the bottom-left corner
                [Que, GoalReached] = Move(-1,-1,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move down & left
                if GoalReached
                    break
                end
            end
            if CurrentNode(1) > 1  % if not on the left edge
                [Que, GoalReached] = Move(-1,0,CurrentNode, CurrentNodeInfo, METHOD, Que, weight, res); % move left
                if GoalReached
                    break
                end
            end
        end

        if CurrentNodeInfo(1,1,1) > 2
            % establish block
            bX = [-0.5,-0.5,0.5,0.5,-0.5]*res;
            bY = [-0.5,0.5,0.5,-0.5,-0.5]*res;
            % update color of node to indicate that is has been expanded
            patch(CurrentNode(1)+bX,CurrentNode(2)+bY,'y');
            pause(0.0001);
        end

        % update the Que
        index_old = find(Que(:) == index);
        [Que] = Update_Que_DFS(Que,index_old);

    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Generate Optimal Path
    % the last node in the node set is the terminal node of the solution
    TerminalNodeInfo = NodeSet.NodesInfo(:,:,end);
    TerminalNode = NodeSet.Nodes(:,:,end);

    % lets backtrack through the parent nodes to find the full path
    cost = TerminalNodeInfo(3); % cost is the number of steps taken to get here
    Path(cost+1,:) = TerminalNode;
    PathInfo(cost+1,:) = TerminalNodeInfo; % start at the termpinal node
    
    while cost > 0 % until we have arrived back at the start node
        pN = PathInfo(cost+1,2); % get the parent node number
        pI = find(NodeSet.NodesInfo(:,1,:) == pN); % find the coresponding record index
        PathInfo(cost,:) = NodeSet.NodesInfo(:,:,pI); % save the parent node's info
        Path(cost,:) = NodeSet.Nodes(:,:,pI); % save the path
        patch(Path(cost,1)+bX,Path(cost,2)+bY,'r');
        pause(0.0001);
        cost = cost - 1; % move on to that parent node
    end  

%     plot(Path(:,1),Path(:,2),'k','LineWidth',2);
%     x1 = num2str(StartNode(1));y1 = num2str(StartNode(2));
%     x2 = num2str(GoalNode(1));y2 = num2str(GoalNode(2));
%     txt = strcat(x1,',',y1,'_',x2,',',y2,'_','Path2.jpg');
%     saveas(gcf,txt);
%     axis([min(Path(:,1))-5,max(Path(:,1))+5,min(Path(:,2))-5,max(Path(:,2))+2])
%     txt2 = strcat(x1,',',y1,'_',x2,',',y2,'_','Path_Zoom2.jpg');
%     saveas(gcf,txt2);

end
