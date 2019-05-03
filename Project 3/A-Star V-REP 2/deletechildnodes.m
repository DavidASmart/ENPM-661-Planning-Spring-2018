function [Que] = deletechildnodes(nn,Que)
% delete all the children of a certain node

global NodeSet
global np
global n

% delete its children & remove from Que
[~,ix] = find(NodeSet.NodesInfo(1,2,:) == nn); % identify children
nn2 = NodeSet.NodesInfo(:,1,ix);
if ~isempty(ix)
    
    % deleted from display
    delete(np(ix))
    delete(n(ix))
    % delete from record
    NodeSet.NodesInfo(:,:,ix) = [];
    NodeSet.Nodes(:,:,ix) = [];
    % delete from Que
    ixQ = find(Que(:) == ix);
    Que(ixQ) = [];
    
end

% delete its "grand-children" too
if ~isempty(nn2)
    [~,ix2] = find(NodeSet.NodesInfo(1,2,:) == nn2); % identify grand-children
    nn3 = NodeSet.NodesInfo(:,1,ix2);
    if ~isempty(ix2)
        % deleted from display
        delete(np(ix2))
        delete(n(ix2))
        % delete from record
        NodeSet.NodesInfo(:,:,ix2) = [];
        NodeSet.Nodes(:,:,ix2) = [];
        % delete from Que
        ixQ2 = find(Que(:) == ix2);
        Que(ixQ2) = [];
    end

    % delete its "great-grand-children"
    if ~isempty(nn3)
        [~,ix3] = find(NodeSet.NodesInfo(1,2,:) == nn3); % identify grand-children
        if ~isempty(ix3)
            % deleted from display
            delete(np(ix3))
            delete(n(ix3))
            % delete from record
            NodeSet.NodesInfo(:,:,ix3) = [];
            NodeSet.Nodes(:,:,ix3) = [];
            % delete from Que
            ixQ3 = find(Que(:) == ix3);
            Que(ixQ3) = [];
        end
    end
end

