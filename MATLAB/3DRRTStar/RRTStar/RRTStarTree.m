classdef RRTStarTree
    %RRTSTAR Class containing the Tree of the search domain and methods to operate on it
    %   Detailed explanation goes here
    
    properties
        nodes = [];
        edges = []; % [Parent,child] --> Rewire changes parent
        costs = [];
        endNode = [];
        endNodeIndex;
        endBias = 0.1;
        numEdges 
        numNodes
        
        SamplingRestrictionCheck
        SamplingRestrictionDistance = 0.3 % meters

        Path = [];
        sizePath = [];
        
        % Visualization Tools, Comment out for memory
        TreePlot
        PathPlot
    end
    
    methods
        function Tree = RRTStarTree(startNode,endNode)
            %RRTSTAR Construct an instance of this class
            %   Creates an object that contains the RRT* tree and the
            %   methods to manipulate it

            Tree.edges = [];
            Tree.costs(1) = 0;
            Tree.nodes(1,:) = startNode;
            Tree.endNode = endNode;
            Tree.numEdges = 0;
            Tree.numNodes = 1;

            Tree.SamplingRestrictionCheck = 0;
        end
        
        function Tree = AddNode(Tree,Grid,newNode,parentNodeIndex)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            Tree.numNodes = Tree.numNodes + 1;
            Tree.nodes(Tree.numNodes,:) = newNode;
            Tree.numEdges = Tree.numEdges + 1; 
            Tree.edges(Tree.numEdges,:) = [parentNodeIndex,Tree.numNodes];
            Tree.costs(Tree.numNodes)  = Tree.CostFunction(Grid,newNode,parentNodeIndex);
        end
        
        function sampledNode = SampleNewNode(Tree,Grid)
            % This function Samples a new node in the 
            NodeGenerated = 0;
            while NodeGenerated ~=1
                
                % normal sampling process
                if rand < Tree.endBias
                    sampledNode = Tree.endNode;
                    isEndNode = 1;
                    NodeGenerated = 1;
                % sample
                else
                sampledNode = [rand*Grid.indexDimensions(1), rand*Grid.indexDimensions(2), rand*Grid.indexDimensions(3)];
                isEndNode = 0;
                end

                if 1 ~= Grid.ContainsObstacle(sampledNode) && isEndNode ~= 1
                    NodeGenerated = 1;
                    sampledNode = Grid.getPoint(sampledNode);
                end
                
            end
            
        end

        function cost = CostFunction(Tree,Grid,Node,ParentNodeindex)
            dist = sqrt( (Node(1) - Tree.nodes(ParentNodeindex,1))^2 + (Node(2) - Tree.nodes(ParentNodeindex,2))^2);
            cost = Tree.costs(ParentNodeindex) + dist;

        end

        function Tree = Rewire(Tree,Grid,SeedNodeIndex,nearNodeIndexes)

            % The seed node is the most recently added node
            numchecks = size(nearNodeIndexes);
            for i = 1:numchecks(2)
                tempcost = Tree.CostFunction(Grid,Tree.nodes(nearNodeIndexes(i),:),Tree.numNodes);

                if tempcost < Tree.costs(nearNodeIndexes(i))
                    % trigger rewire
                    I = find(Tree.edges(:,2) == nearNodeIndexes(i));
                    Tree.edges(I,:) = [Tree.numNodes,nearNodeIndexes(i)];
                    Tree.costs(nearNodeIndexes(i)) = tempcost;

                    % % Debug
                    % if Tree.SamplingRestrictionCheck == 1
                    %     disp("Rewired close node")
                    % end

                end
                
            end

        end

        function Index = FindNearestNode(Tree,Node)
            ManhattanDistancesSqrd = zeros(Tree.numNodes,1);
            for i = 1:Tree.numNodes
                ManhattanDistancesSqrd(i) = ((Node(1) - Tree.nodes(i,1))^2 + (Node(2) - Tree.nodes(i,2))^2 + (Node(3) - Tree.nodes(i,3))^2);
            end
            [~,Index] = min(ManhattanDistancesSqrd);
        end
        
        function Tree = PlotTree(Tree,Grid)
            for i = 1:Tree.numEdges
                index1 = Grid.getIndex(Tree.nodes(Tree.edges(i,1),:));
                index2 = Grid.getIndex(Tree.nodes(Tree.edges(i,2),:));
                Tree.TreePlot = plot3([index1(1), index2(1)],[index1(2), index2(2)],[index1(3), index2(3)],'r');
                hold on
            end
        end

        function Tree = PlotPath(Tree,Grid)
            for i = 1:Tree.sizePath - 1
                index1 = Grid.getIndex(Tree.nodes(Tree.Path(i),:));
                index2 = Grid.getIndex(Tree.nodes(Tree.Path(i + 1),:));
                Tree.PathPlot = plot3([index1(1), index2(1)],[index1(2), index2(2)],[index1(3), index2(3)],'g',"LineWidth",2);
                hold on
            end
        end

        function Tree = setPath(Tree)
            % can only be called once the end node has been found
            currentNode = Tree.endNodeIndex;
            count = 1;

            while currentNode ~= 1
                path(count) = currentNode;
                    ParentNodeEdgeIndex = find(Tree.edges(:,2) == currentNode);
                currentNode = Tree.edges(ParentNodeEdgeIndex,1);
                count = count + 1;
            end
            path(count) = currentNode;
            Tree.Path = path;
            PathSize = size(Tree.Path);
            Tree.sizePath = PathSize(2);
        end
    end
end
        % helper funciton to clean up sampling code
        function bool = isPointNearLine(Point, Center1, Center2, Radius)
            CenterlineVector = Center2 - Center1;
            Center2 = Center2 + CenterlineVector./norm(CenterlineVector);
            Center1 = Center1 - CenterlineVector./norm(CenterlineVector);
            CenterlineVector = Center2 - Center1;
            
            VectorProjection = (dot(Point,CenterlineVector)/dot(CenterlineVector,CenterlineVector))*CenterlineVector;
            ProjectedPoint = Center1 + VectorProjection;
            % check ranges of the 
            xMin = min(Center1(1),Center2(1));
            xMax = max(Center1(1),Center2(1));
            yMin = min(Center1(2),Center2(2));
            yMax = max(Center1(2),Center2(2));
            zMin = min(Center1(3),Center2(3));
            zMax = max(Center1(3),Center2(3));
            if (ProjectedPoint(1) > xMin && ProjectedPoint(1) < xMax && ProjectedPoint(2) > yMin && ProjectedPoint(2) < yMax && ProjectedPoint(3) > zMin && ProjectedPoint(3) < zMax)
                if norm(ProjectedPoint - Point) <= Radius
                    bool = 1;
                else
                    bool = 0;
                end
            else
                bool = 0;
            end
        end
