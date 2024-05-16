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
            % This function Samples a new node in the 
            NodeGenerated = 0;
            while NodeGenerated ~=1
                
                % normal sampling process
                if rand < Tree.endBias
                    % "sample" the end point
                    sampledNode = Tree.endNode;
                    isEndNode = 1;
                    NodeGenerated = 1;
                else
                    % sample a random point
                sampledNode = [rand*Grid.indexDimensions(1), rand*Grid.indexDimensions(2), rand*Grid.indexDimensions(3)];
                isEndNode = 0;
                end
                % check if its in an obstacle, if it fails this check the loop continues and a new node is sampled
                if 1 ~= Grid.ContainsObstacle(sampledNode) && isEndNode ~= 1
                    NodeGenerated = 1;
                    sampledNode = Grid.getPoint(sampledNode);
                end
                
            end
            
        end

        function cost = CostFunction(Tree,Node,ParentNodeindex)
            % Cost function, current implementation only accounts for distance, but can be modified easily
            dist = sqrt( (Node(1) - Tree.nodes(ParentNodeindex,1))^2 + (Node(2) - Tree.nodes(ParentNodeindex,2))^2);
            cost = Tree.costs(ParentNodeindex) + dist;

        end

        function Tree = Rewire(Tree,nearNodeIndexes)
            % rewire function recieves a list of nodes that are near the newly sampled node, 
            % we check if making the new node the parent of these new ones would reduce their
            %  costs and if so we rewire the tree to make this the case

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
