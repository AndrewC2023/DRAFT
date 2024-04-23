classdef RRTStar
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
        SamplingRestrictionRegions = []; % four point Tree.SampilingRestrictionRegionss
        SamplingRestrictionRegionAreas = []
        SamplingRestrictionDistance = 0.2 % meters

        Path = [];
        sizePath = [];
        
        % Visualization Tools, Comment out for memory
        TreePlot
        PathPlot
    end
    
    methods
        function Tree = RRTStar(startNode,endNode)
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
            Tree.SamplingRestrictionRegions = [];
            
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
                if Tree.SamplingRestrictionCheck == 1
                    % sample
                    sampledNode = [rand*Grid.indexDimensions(1), rand*Grid.indexDimensions(2)];
                    % check if we're in an obstacle
                    if 1 ~= Grid.ContainsObstacle(sampledNode)
                        % region Check
                        sampledNodePoint = Grid.getPoint(sampledNode);
                        for i = 1:Tree.sizePath - 1
                            % Point R1 R2
                            A1 = abs(0.5*( sampledNodePoint(1)*(Tree.SamplingRestrictionRegions(1,2,i) - Tree.SamplingRestrictionRegions(2,2,i))...
                                           + Tree.SamplingRestrictionRegions(1,1,i)*(Tree.SamplingRestrictionRegions(2,2,i) - sampledNodePoint(2))...
                                           + Tree.SamplingRestrictionRegions(2,1,i)*(sampledNodePoint(2) - Tree.SamplingRestrictionRegions(1,2,i))) );
                            % Point R2 R3
                            A2 = abs(0.5*( sampledNodePoint(1)*(Tree.SamplingRestrictionRegions(2,2,i) - Tree.SamplingRestrictionRegions(3,2,i))...
                                           + Tree.SamplingRestrictionRegions(2,1,i)*(Tree.SamplingRestrictionRegions(3,2,i) - sampledNodePoint(2))...
                                           + Tree.SamplingRestrictionRegions(3,1,i)*(sampledNodePoint(2) - Tree.SamplingRestrictionRegions(2,2,i))) );
                            % Point R3 R4
                            A3 = abs(0.5*( sampledNodePoint(1)*(Tree.SamplingRestrictionRegions(3,2,i) - Tree.SamplingRestrictionRegions(4,2,i))...
                                           + Tree.SamplingRestrictionRegions(3,1,i)*(Tree.SamplingRestrictionRegions(4,2,i) - sampledNodePoint(2))...
                                           + Tree.SamplingRestrictionRegions(4,1,i)*(sampledNodePoint(2) - Tree.SamplingRestrictionRegions(3,2,i))) );
                            % Point R4 R1
                            A4 = abs(0.5*( sampledNodePoint(1)*(Tree.SamplingRestrictionRegions(4,2,i) - Tree.SamplingRestrictionRegions(1,2,i))...
                                           + Tree.SamplingRestrictionRegions(4,1,i)*(Tree.SamplingRestrictionRegions(1,2,i) - sampledNodePoint(2))...
                                           + Tree.SamplingRestrictionRegions(1,1,i)*(sampledNodePoint(2) - Tree.SamplingRestrictionRegions(4,2,i))) );
                            CheckArea = A1 + A2 + A3 + A4;

                            if CheckArea < 1.05*Tree.SamplingRestrictionRegionAreas(i)
                                NodeGenerated = 1;
                                sampledNode = sampledNodePoint;
                                break
                                
                            end
                        end
                            % debug: Visualize where sampling occurs
                            % figure(1)
                            %     scatter(sampledNode(1),sampledNode(2),60,"x")
                            %     hold on
                    end
                    
                else
                    % add end Bias
                    if rand < Tree.endBias
                        sampledNode = Tree.endNode;
                        isEndNode = 1;
                        NodeGenerated = 1;
                    % sample
                    else
                    sampledNode = [rand*Grid.indexDimensions(1), rand*Grid.indexDimensions(2)];
                    isEndNode = 0;
                    end

                    if 1 ~= Grid.ContainsObstacle(sampledNode) && isEndNode ~= 1
                        NodeGenerated = 1;
                        sampledNode = Grid.getPoint(sampledNode);
                    end
                end
            end

        end

        function Tree = GenerateSampleRegions(Tree)

            % calculate the vectors
            expansionPoints = zeros(Tree.sizePath,2,2);
            for i = 1:Tree.sizePath

                if i == 1 % first Point
                    pathVector = Tree.nodes(Tree.Path(i + 1),:) - Tree.nodes(Tree.Path(i),:);
                    unitPathVector = pathVector./(norm(pathVector));

                    expansionPoints(i,:,1) = Tree.SamplingRestrictionDistance*([-unitPathVector(2) - unitPathVector(1) , unitPathVector(1) - unitPathVector(2)]);
                    expansionPoints(i,:,2) = Tree.SamplingRestrictionDistance*([unitPathVector(2) - unitPathVector(1) ,- unitPathVector(1) - unitPathVector(2)]);

                    expansionPoints(i,:,1) = expansionPoints(i,:,1) + Tree.nodes(Tree.Path(i),:);
                    expansionPoints(i,:,2) = expansionPoints(i,:,2) + Tree.nodes(Tree.Path(i),:);
                elseif i == Tree.sizePath % last Point
                    pathVector = Tree.nodes(Tree.Path(i - 1),:) - Tree.nodes(Tree.Path(i),:);
                    unitPathVector = pathVector./(norm(pathVector));
                    
                    expansionPoints(i,:,2) = Tree.SamplingRestrictionDistance*([-unitPathVector(2) - unitPathVector(1) , unitPathVector(1) - unitPathVector(2)]);
                    expansionPoints(i,:,1) = Tree.SamplingRestrictionDistance*([unitPathVector(2) - unitPathVector(1) , -unitPathVector(1) - unitPathVector(2)]);
                    
                    expansionPoints(i,:,1) = expansionPoints(i,:,1) + Tree.nodes(Tree.Path(i),:);
                    expansionPoints(i,:,2) = expansionPoints(i,:,2) + Tree.nodes(Tree.Path(i),:);
                else % middle Points
                    forwardVector = [(Tree.nodes(Tree.Path(i + 1),1) - Tree.nodes(Tree.Path(i),1)),(Tree.nodes(Tree.Path(i + 1),2) - Tree.nodes(Tree.Path(i),2))];
                    backwardVector = [(Tree.nodes(Tree.Path(i - 1),1) - Tree.nodes(Tree.Path(i),1)),(Tree.nodes(Tree.Path(i - 1),2) - Tree.nodes(Tree.Path(i),2))];

                    magnitudeBackwards = (norm(backwardVector));
                    magnitudeForwards = (norm(forwardVector));
                    unitExpansionVector = (backwardVector./magnitudeBackwards + forwardVector./magnitudeForwards);
                    unitExpansionVector = unitExpansionVector./norm(unitExpansionVector);
                    sintheta = (forwardVector(1)*unitExpansionVector(2) - (unitExpansionVector(1)*forwardVector(2)))/(magnitudeForwards);
                    dist = Tree.SamplingRestrictionDistance/sintheta;
                    ExpansionVector = unitExpansionVector*dist;
                    
                    expansionPoints(i,:,1) = Tree.nodes(Tree.Path(i),:) + ExpansionVector;
                    expansionPoints(i,:,2) = Tree.nodes(Tree.Path(i),:) - ExpansionVector;
                    
                end

            end
            % Debug
                % for i = 1:Tree.sizePath
                %     point2Plot1 = Grid.getIndex(expansionPoints(i,:,1));
                %     point2Plot2 = Grid.getIndex(expansionPoints(i,:,2));
                %     figure(1)
                %         scatter(point2Plot1(1),point2Plot1(2))
                %         scatter(point2Plot2(1),point2Plot2(2))
                % end
            %
            % use the vectors to generate the zones
            SampleRegions = zeros(4,2,Tree.sizePath - 1);
            Tree.SamplingRestrictionRegionAreas = zeros(Tree.sizePath - 1,1);
            for i = 1:(Tree.sizePath - 1)
                SampleRegions(1,:,i) = expansionPoints(i,:,1);
                SampleRegions(2,:,i) = expansionPoints(i + 1,:,1);
                SampleRegions(3,:,i) = expansionPoints(i + 1,:,2);
                SampleRegions(4,:,i) = expansionPoints(i,:,2);

                % Debug
                    % figure(1)
                    %     plot(((SampleRegions(:,1,i) + (1/2)*Grid.Resolution)/Grid.Resolution),(SampleRegions(:,2,i) + (1/2)*Grid.Resolution)/Grid.Resolution);
                    %     hold on

                Tree.SamplingRestrictionRegionAreas(i) = abs((0.5)*((SampleRegions(1,1,i)*SampleRegions(2,2,i)...
                                                           + SampleRegions(2,1,i)*SampleRegions(3,2,i)...
                                                           + SampleRegions(3,1,i)*SampleRegions(4,2,i)...
                                                           + SampleRegions(4,1,i)*SampleRegions(1,2,i))...
                                                           - (SampleRegions(2,1,i)*SampleRegions(1,2,i)...
                                                           + SampleRegions(3,1,i)*SampleRegions(2,2,i)...
                                                           + SampleRegions(4,1,i)*SampleRegions(3,2,i)...
                                                           + SampleRegions(1,1,i)*SampleRegions(4,2,i))));
            end

            Tree.SamplingRestrictionRegions = SampleRegions;

        end
        
        function cost = CostFunction(Tree,Grid,Node,ParentNodeindex)
            dist = sqrt( (Node(1) - Tree.nodes(ParentNodeindex,1))^2 + (Node(2) - Tree.nodes(ParentNodeindex,2))^2);
            near_obs_count = 0;

            % Obstacle Proximity
            % Check_Points =  [ 1, 0;
            %                  -1, 0;
            %                   1, 1;
            %                  -1,-1;
            %                   0, 1;
            %                   0,-1;
            %                   1,-1;
            %                  -1, 1;
            %                   2, 0;
            %                   2, 1;
            %                   2,-1;
            %                   2, 2;
            %                   2,-2;
            %                   1, 2;
            %                   1,-2;
            %                   0, 2;
            %                   0,-2;
            %                  -1, 2;
            %                  -1,-2;
            %                  -2, 0;
            %                  -2, 1;
            %                  -2,-1;
            %                  -2, 2;
            %                  -2,-2];

            % Rounded = round(Grid.getIndex(Node));
            % for i = 1:24
            %     Check = Rounded + Check_Points(i,:);
            %     if Check(1) <= 0 || Check(1) >= (Grid.indexDimensions(1) + 1) || Check(2) <= 0 || Check(2) >= (Grid.indexDimensions(2) + 1)
            %         % dont check
            %     else
            %         if Grid.Occupancy(Check(1),Check(2)) == 1
            %             near_obs_count = near_obs_count + 1;
            %         end
            %     end
        
            % end

            cost = Tree.costs(ParentNodeindex) + dist + near_obs_count;

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
                ManhattanDistancesSqrd(i) = ((Node(1) - Tree.nodes(i,1))^2 + (Node(2) - Tree.nodes(i,2))^2);
            end
            [~,Index] = min(ManhattanDistancesSqrd);
        end
        
        function Tree = PlotTree(Tree,Grid)
            for i = 1:Tree.numEdges
                Tree.TreePlot = plot(Grid.getIndex([Tree.nodes(Tree.edges(i,1),1),Tree.nodes(Tree.edges(i,2),1)]),Grid.getIndex([Tree.nodes(Tree.edges(i,1),2),Tree.nodes(Tree.edges(i,2),2)]),'r');
                hold on
            end
        end

        function Tree = PlotPath(Tree,Grid)
            for i = 1:Tree.sizePath - 1
                Tree.PathPlot = plot(Grid.getIndex([Tree.nodes(Tree.Path(i),1),Tree.nodes(Tree.Path(i + 1),1)]),Grid.getIndex([Tree.nodes(Tree.Path(i),2),Tree.nodes(Tree.Path(i + 1),2)])...
                    ,'g',"LineWidth",2);
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

