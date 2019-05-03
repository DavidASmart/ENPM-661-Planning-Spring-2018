function [Que] = UpdateQueAndDisplay(NewNode, NewNodeInfo, Que, Visited, Crash, Goal, x, y, res)

    global NodeSet
    global n
    global np

    % establish block
    th = 0:15:360;
    bX = (res/4)*cosd(th);
    bY = (res/4)*sind(th);

    % Add NewNode to record and Que
    if ~Crash % if not going to crash
        
        if Visited(1) == 0 % if not already visited
            
            % add to record
            NodeSet.Nodes(:,:,end+1) = NewNode;
            NodeSet.NodesInfo(:,:,end+1) = NewNodeInfo;
 
            % plot
            %np(NewNodeInfo(1)) = plot(x,y,'b','LineWidth',2);
            %n(NewNodeInfo(1)) = patch(NewNode(1)+bX,NewNode(2)+bY,'g'); % plot node
            axis equal
            if ~Goal
                Que(end+1) = NewNodeInfo(1);    % add to the Que to be expanded
            end
            
        else % if already visited
            [~,ixV] = find(NodeSet.NodesInfo(1,1,:) == Visited(:)); % find indecies of equivilent nodes
            if NewNodeInfo(3) < min(NodeSet.NodesInfo(1,3,ixV)) % if new is a lower cost

                
                for i = 1:length(Visited) % for all "similar nodes"
                    % deleted old node & path
                    %delete(np(Visited(i)))
                    %delete(n(Visited(i)))
                    
                    % because the nodes do not always perfectly line up
                    % all of its children must be deleted
                    deletechildnodes(Visited(i),Que);
                end
                
                for i = length(ixV):-1:1 % for all "similar nodes"
                    
                    % delete the old node from the record
                    NodeSet.Nodes(:,:,ixV(i)) = [];
                    NodeSet.NodesInfo(:,:,ixV(i)) = [];
                    
                    % add to record
                    NodeSet.Nodes(:,:,end+1) = NewNode;
                    NodeSet.NodesInfo(:,:,end+1) = NewNodeInfo;

                     % plot
                    %np(NewNodeInfo(1)) = plot(x,y,'b','LineWidth',2);
                    %n(NewNodeInfo(1)) = patch(NewNode(1)+bX,NewNode(2)+bY,'g'); % plot node
                    axis equal
                    if ~Goal
                        Que(end+1) = NewNodeInfo(1);    % add to the Que to be expanded
                    end
                    
                    
                end
            end
        end
    end
    
    
end

