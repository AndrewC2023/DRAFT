classdef Edge
    %EDG Summary of this class goes here
    %   Detailed explanation goes here
    
    properties

        Edges
            % Edge Structure: [node1 , node2, distance between nodes]

        Max_Edge_Length
            
    end
    
    methods
        function Edge = Create_New_Edge(Edge,Parent_Node,Target_Node,distance)
            % This is the operation to generate a new edge to be stored
            % based off of the two nodes it is connecting 
            % to save on later computational costs, we will also store the 
            % distance cost of the edges in the third slot
            a = size(Edge.Edges);
            Edge.Edges(a(1)+1,:) = [Parent_Node,Target_Node,distance];
        end
        
        function Edge = Rewire(Edge,Valid_Near_Nodes,Cost_0,Node_Class,Obstacles,MAP_Size,New_Node_Num)
            % we start with the current new node and check a new calculated cost for all neighbors
            a = size(Valid_Near_Nodes);
            for i = 1:a(1)
                Check_Node = Node_Class.Nodes(Valid_Near_Nodes(i,1),:);
                dist_New = sqrt( (Node_Class.Nodes(New_Node_Num,1) - Node_Class.Nodes(Valid_Near_Nodes(i,1),1))^2 ...
                        + (Node_Class.Nodes(New_Node_Num,2) - Node_Class.Nodes(Valid_Near_Nodes(i,1),2))^2);
                Temp_Cost = Cost(Check_Node,dist_New,Obstacles,Cost_0,MAP_Size);
                if Temp_Cost < Node_Class.Node_Cost(Valid_Near_Nodes(i,1))
                    % This node needs to be rewired
                    % find the edge to remove
                    ind = find(Edge.Edges(:,2) == Valid_Near_Nodes(i,1));
                    % rewire it
                    Node_Class.Node_Cost(Valid_Near_Nodes(i,1)) = Temp_Cost;

                    Edge.Edges(ind,1) = New_Node_Num;
                    Edge.Edges(ind,3) = dist_New;

                    % now create a set of items that have been rewired, and we will check their neighbors to get rewired
                end
            end
        end
    end
end

