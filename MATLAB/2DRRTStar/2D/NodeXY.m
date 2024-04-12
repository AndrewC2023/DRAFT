classdef NodeXY
    %NODE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Nodes = [];
            % Each node contains the coordinates [x,y]
        Num_Nodes
        Node_Cost = [];
            % The cost calculated to run to each node.
            % the row of this array must match the row of the nodes array
            % to remain consistent thus weight of Nodes(i,:) is Node_Cost(i)
        End_Node = [];
            % This is the node that is both within the end region and 
            % closest to the true end point
        Parent_Node = [];
            % This is the nukber of the node that is the parent of the node that corresponds to the index of this array
    end
    methods
        function NodeXY = NodeGeneration(NodeXY,Target_Node)
            
            NodeXY.Nodes(NodeXY.Num_Nodes(1) + 1,:) = Target_Node;

            NodeXY.Num_Nodes = size(NodeXY.Nodes);
        end

        function NodeXY = ParentNodeGeneration(NodeXY,Edge_Class)
            NodeXY.Parent_Node = zeros(NodeXY.Num_Nodes(1)-2,1);
            for i = 1:NodeXY.Num_Nodes
                NodeXY.Parent_Node(Edge_Class.Edges(i,2)) = Edge_Class.Edges(i,1);
            end
            
        end
        function NodeXY = Nodal_Cost(NodeXY,targetNode,currentNode)

            % cost calculation within the Node class
        end
    end
end

