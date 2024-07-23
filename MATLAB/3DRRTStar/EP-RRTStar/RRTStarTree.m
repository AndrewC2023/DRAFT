classdef RRTStarTree
    %RRTSTAR Class containing the Tree of the search domain and methods to operate on it
    %   Detailed explanation goes here
    
    properties
        nodes = []; % the list of Nodes in true space (Point) with each row being a node (node index corresponds to the order it was added to the tree)
        edges = []; % [Parent,child] ~ Rewire function only changes parent ~ 
        costs = []; % the lists of cost for each node, each index lines up with corresponding node index
        endNode = []; % end node in true space
        endNodeIndex; % end node in Index (occupancy grid) space
        % the proportion of times the end point is sampled when running the normal RRT* Algorithm, 
        % this stops when the initial path is found and the sample restriction begins
        endBias = 0.1; 
        numEdges {int32}
        numNodes {int32}
        
        SamplingRestrictionCheck {logical}
        SamplingRestrictionDistance = 0.1 % meters

        Path = [];
        sizePath = [];
        
        % Visualization Tools, Comment out for memory
        TreePlot
        PathPlot
    end
    
    methods
        function Tree = RRTStarTree(startNode,endNode)
            % RRTSTAR Constructs an instance of this class
            %   Creates an object that contains the RRT* tree and the
            %   methods to manipulate the tree

            Tree.edges = [];
            Tree.costs(1) = 0;
            Tree.nodes(1,:) = startNode;
            Tree.endNode = endNode;
            Tree.numEdges = 0;
            Tree.numNodes = 1;

            Tree.SamplingRestrictionCheck = false;
        end
        
        function Tree = AddNode(Tree,newNode,parentNodeIndex)
            % Appends the fed node to the node list in the class and assigns the edge with the fed parent as well
            Tree.numNodes = Tree.numNodes + 1;
            Tree.nodes(Tree.numNodes,:) = newNode;
            Tree.numEdges = Tree.numEdges + 1; 
            Tree.edges(Tree.numEdges,:) = [parentNodeIndex,Tree.numNodes];
            Tree.costs(Tree.numNodes)  = Tree.CostFunction(newNode,parentNodeIndex);
        end
        
        function sampledNode = SampleNewNode(Tree,Grid)
            % This function Samples a new node in the obstacle free space
            NodeGenerated = false;
            while NodeGenerated ~= true
                % check if we are restricting our sampling process
                if Tree.SamplingRestrictionCheck == true
                    % in restricted sampling process

                    % sample a random point
                    sampledNode = [rand*Grid.indexDimensions(1), rand*Grid.indexDimensions(2), rand*Grid.indexDimensions(3)];

                    % check if its in an obstacle, if it fails this check the loop continues and a new node is sampled
                    if 1 ~= Grid.ContainsObstacle(sampledNode)

                        sampledNodePoint = Grid.getPoint(sampledNode);

                        % find the nearest path point, so we only check two cylinders (can be done without a sqrt saving computation)
                        ManhattanDistancesSqrd = zeros(Tree.sizePath,1);
                        for i = 1:Tree.sizePath
                            ManhattanDistancesSqrd(i) = ((sampledNodePoint(1) - Tree.nodes(Tree.Path(i),1))^2 + (sampledNodePoint(2) - Tree.nodes(Tree.Path(i),2))^2 + (sampledNodePoint(3) - Tree.nodes(Tree.Path(i),3))^2);
                        end 
                        [~,Index] = min(ManhattanDistancesSqrd);

                        % check what path index is closest, the end points of the path are treated somewhat differently than the middle points
                        if Index == 1
                            % CASE 1
                            % nearest to start of the path
                            if norm(sampledNodePoint - Tree.nodes(Tree.Path(Index),:)) < Tree.SamplingRestrictionDistance
                                % we are near enough to the end node that we can confirm this as a valid sample
                                NodeGenerated = true;
                                sampledNode = sampledNodePoint;
                            else
                                % check if we are within a radius of the path cylinder coming off the end
                                nearLine = isPointNearLine(sampledNodePoint, Tree.nodes(Tree.Path(Index),:),Tree.nodes(Tree.Path(Index + 1),:),Tree.SamplingRestrictionDistance);
                                if nearLine
                                    NodeGenerated = true;
                                    sampledNode = sampledNodePoint;
                                end
                            end
    
                        elseif Index == Tree.sizePath
                            % CASE 2
                            % nearest to end of the path
                            if norm(sampledNodePoint - Tree.nodes(Tree.Path(Index),:)) < Tree.SamplingRestrictionDistance
                                % we are near the end node
                                NodeGenerated = true;
                                sampledNode = sampledNodePoint;
                            else
                                % check if we are within a radius of the path cylinder coming off the end
                                nearLine = isPointNearLine(sampledNodePoint, Tree.nodes(Tree.Path(Index),:),Tree.nodes(Tree.Path(Index - 1),:),Tree.SamplingRestrictionDistance);
                                if nearLine
                                    NodeGenerated = true;
                                    sampledNode = sampledNodePoint;
                                end
                            end
    
                        else
                            % we are closest to one of the middle points
                            % here we check the cylinders behind and in front
                            nearLineForward = isPointNearLine(sampledNodePoint, Tree.nodes(Tree.Path(Index),:),Tree.nodes(Tree.Path(Index + 1),:),Tree.SamplingRestrictionDistance);
                            if nearLineForward
                                NodeGenerated = true;
                                sampledNode = sampledNodePoint;
                            else
                                nearLineBackwards = isPointNearLine(sampledNodePoint, Tree.nodes(Tree.Path(Index),:),Tree.nodes(Tree.Path(Index + 1),:),Tree.SamplingRestrictionDistance);
                                if nearLineBackwards
                                    NodeGenerated = true;
                                    sampledNode = sampledNodePoint;
                                end
                            end
                        end

                    end
                    
                else
                    % normal sampling process
                    % add end Bias
                    if rand < Tree.endBias
                        % "sample" the end point
                        sampledNode = Tree.endNode;
                        isEndNode = true;
                        NodeGenerated = true;
                    else
                        % sample a random point
                        sampledNode = [rand*Grid.indexDimensions(1), rand*Grid.indexDimensions(2), rand*Grid.indexDimensions(3)];
                        isEndNode = false;
                    end
                    % check if its in an obstacle, if it fails this check the loop continues and a new node is sampled
                    if 1 ~= Grid.ContainsObstacle(sampledNode) && isEndNode ~= true
                        NodeGenerated = true;
                        sampledNode = Grid.getPoint(sampledNode);
                    end
                end
            end
            
        end

        function cost = CostFunction(Tree,Node,ParentNodeindex)
            % Cost function, current implementation only accounts for distance, but can be modified easily
            dist = sqrt( (Node(1) - Tree.nodes(ParentNodeindex,1))^2 + (Node(2) - Tree.nodes(ParentNodeindex,2))^2 + (Node(3) - Tree.nodes(ParentNodeindex,3))^2);
            cost = Tree.costs(ParentNodeindex) + dist;
        end

        function Tree = Rewire(Tree,nearNodeIndexes)
            % rewire function recieves a list of nodes that are near the newly sampled node, 
            % we check if making the new node the parent of these new ones would reduce their
            % costs and if so we rewire the tree to make this the case

            % The seed node is the most recently added node
            numchecks = size(nearNodeIndexes);
            for i = 1:numchecks(2)
                tempcost = Tree.CostFunction(Tree.nodes(nearNodeIndexes(i),:),Tree.numNodes);

                if tempcost < Tree.costs(nearNodeIndexes(i))
                    % trigger rewire
                    I = find(Tree.edges(:,2) == nearNodeIndexes(i));
                    Tree.edges(I,:) = [Tree.numNodes,nearNodeIndexes(i)];
                    Tree.costs(nearNodeIndexes(i)) = tempcost;

                end
                
            end

        end

        % helper to find the nearest node in the tree to some point fed to the function
        function Index = FindNearestNode(Tree,Node)
            % using sqared manhattan distances allows us to avoid using a square root
            ManhattanDistancesSqrd = zeros(Tree.numNodes,1);
            for i = 1:Tree.numNodes
                ManhattanDistancesSqrd(i) = ((Node(1) - Tree.nodes(i,1))^2 + (Node(2) - Tree.nodes(i,2))^2 + (Node(3) - Tree.nodes(i,3))^2);
            end
            [~,Index] = min(ManhattanDistancesSqrd);
        end
        
        % helper to visualize the tree
        function Tree = PlotTree(Tree,Grid)
            for i = 1:Tree.numEdges
                index1 = Grid.getIndex(Tree.nodes(Tree.edges(i,1),:));
                index2 = Grid.getIndex(Tree.nodes(Tree.edges(i,2),:));
                Tree.TreePlot = plot3([index1(1), index2(1)],[index1(2), index2(2)],[index1(3), index2(3)],'r');
                hold on
            end
        end

        % helper to visualize the path
        function Tree = PlotPath(Tree,Grid)
            for i = 1:Tree.sizePath - 1
                index1 = Grid.getIndex(Tree.nodes(Tree.Path(i),:));
                index2 = Grid.getIndex(Tree.nodes(Tree.Path(i + 1),:));
                Tree.PathPlot = plot3([index1(1), index2(1)],[index1(2), index2(2)],[index1(3), index2(3)],'g',"LineWidth",2);
                hold on
            end
        end

        % helper that sets the path for the class by back tracing parents from the end node
        % this needs to be called before the sampling restriction process begins
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

% helper funciton to check if a point is within some radius of a line
function bool = isPointNearLine(Point, Center1, Center2, Radius)
    CenterlineVector = Center2 - Center1;
    % extend the line such that if the point is ~1radius of the endpoint it is still valid
    Center2 = Center2 + (CenterlineVector./norm(CenterlineVector))*Radius;
    Center1 = Center1 - (CenterlineVector./norm(CenterlineVector))*Radius;

    CenterlineVector = Center2 - Center1;
    PointVector = Point - Center1;
    
    % project the point onto the centerline vector
    VectorProjection = (dot(PointVector,CenterlineVector)/dot(CenterlineVector,CenterlineVector))*CenterlineVector;
    ProjectedPoint = Center1 + VectorProjection;
    % check range distance to
    xMin = min(Center1(1),Center2(1));
    xMax = max(Center1(1),Center2(1));
    yMin = min(Center1(2),Center2(2));
    yMax = max(Center1(2),Center2(2));
    zMin = min(Center1(3),Center2(3));
    zMax = max(Center1(3),Center2(3));
    if (ProjectedPoint(1) > xMin && ProjectedPoint(1) < xMax && ProjectedPoint(2) > yMin && ProjectedPoint(2) < yMax && ProjectedPoint(3) > zMin && ProjectedPoint(3) < zMax)
        if norm(ProjectedPoint - Point) <= Radius
            % point passed all checks
            bool = true;
        else
            bool = false;
        end
    else
        bool = false;
    end
end
