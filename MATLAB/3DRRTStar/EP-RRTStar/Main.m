clc;
clear;
close all;


%% Initialize

% Inputs 

Constraints.MaxIterations = 3000;
Constraints.MaxEdgeLength = 0.3; % meters
Constraints.GridResolution = 0.05; % meters
Constraints.EndTolerance = 0.05; % meters
Start_End_Indexes = [2,2,2;
                     29,25,15];
h = 30;
w = 30;
z = 30;
Obstacle_Generator3D;
% Construction
Grid = Grid(Constraints.GridResolution,Occupancy');
clear IndexSize Occupancy Constraints.GridResolution

RRTStarTree = RRTStarTree(Grid.getPoint([Start_End_Indexes(1,:)]), ...
    Grid.getPoint([Start_End_Indexes(2,:)]));
clear Start_End_Indexes

iteration = 2;
while iteration <= Constraints.MaxIterations

    Temp.newPointCheck = 0;
    while Temp.newPointCheck ~= 1
        % sample a new point
        Temp.newPoint = RRTStarTree.SampleNewNode(Grid);

        Temp.nearPoint = RRTStarTree.FindNearestNode(Temp.newPoint);
        Temp.nearPoint = RRTStarTree.nodes(Temp.nearPoint,:);

        % steer Function
        if RRTStarTree.SamplingRestrictionCheck ~= 1
            [Temp.newPointCheck,newPointIndexPosition] = steer(Grid,Temp.nearPoint,Temp.newPoint,Constraints.MaxEdgeLength/Grid.Resolution);
        else
            Temp.newPointCheck = 1;
            newPointIndexPosition = Temp.newPoint;
        end
        
    end
    
    newPoint = Grid.getPoint(newPointIndexPosition);
    % wire the new point to the tree
    count = 1;
    for i = 1:RRTStarTree.numNodes
            % TODO: Heavily optimize this step
        if 1.1*Constraints.MaxEdgeLength > sqrt((RRTStarTree.nodes(i,1) - newPoint(1))^2 + (RRTStarTree.nodes(i,2) - newPoint(2))^2 + (RRTStarTree.nodes(i,3) - newPoint(3))^2)

            % 1 means obstacle free
            if 1 == Grid.lineCollisionCheck(Grid.getIndex(RRTStarTree.nodes(i,:)),newPointIndexPosition)
                Temp.nearNodes(count) = i;
                Temp.nearNodeCosts(count) = RRTStarTree.CostFunction(Grid,newPoint,i);
                count = count + 1;
            end

        end

    end

    [~,Temp.ind] = min(Temp.nearNodeCosts);
    Temp.ParentNode = Temp.nearNodes(Temp.ind);

    RRTStarTree = RRTStarTree.AddNode(Grid,newPoint,Temp.ParentNode);
    % now we check if the new point in the end node

    if newPoint(1) == RRTStarTree.endNode(1) && newPoint(2) == RRTStarTree.endNode(2) && newPoint(3) == RRTStarTree.endNode(3)
        RRTStarTree.SamplingRestrictionCheck = 1;
        RRTStarTree.endNodeIndex = RRTStarTree.numNodes;

        % Run the initial Sampling Restriction
        RRTStarTree = RRTStarTree.setPath();
        RRTStarTree = RRTStarTree.GenerateSampleRegions();

        % Debug/Info:
        fprintf("Found End Point after %i iterations \n",iteration)
        fprintf("Current Path Cost: %f \n",RRTStarTree.costs(iteration))
        [~,Temp.PathSize] = size(RRTStarTree.Path);
        fprintf("The current path is: \nIndex: \t X: \t Y: \n")
        for i = 1:Temp.PathSize
            fprintf("%i   \t(%4.3f \t, %4.3f, \t %4.3f) \n",RRTStarTree.Path(i),RRTStarTree.nodes(RRTStarTree.Path(i),1),RRTStarTree.nodes(RRTStarTree.Path(i),2),RRTStarTree.nodes(RRTStarTree.Path(i),3))
        end

        figure(1)
           RRTStarTree = RRTStarTree.PlotTree(Grid);
           RRTStarTree = RRTStarTree.PlotPath(Grid);
           title("Initial Found Path")
           legend([RRTStarTree.TreePlot,RRTStarTree.PathPlot],'Tree','Path')
           hold off

        regenerateCount = 1;
    end

    % rewire step
    RRTStarTree = RRTStarTree.Rewire(Grid,newPoint,Temp.nearNodes);
    clear Temp i newPoint newPointIndexPosition count 

    % If we need to restrict samples we need to get our path and generate
    % sample spaces
    if RRTStarTree.SamplingRestrictionCheck == 1
        regenerateCount = regenerateCount + 1;
        if regenerateCount == 300
            RRTStarTree = RRTStarTree.setPath();
            RRTStarTree = RRTStarTree.GenerateSampleRegions();
            regenerateCount = 1;
            
        end
    end

    iteration = iteration + 1;

end

RRTStarTree = RRTStarTree.setPath();
fprintf("Current Path Cost: %f \n",RRTStarTree.costs(RRTStarTree.endNodeIndex))
[~,Temp.PathSize] = size(RRTStarTree.Path);
fprintf("The Final path is: \nIndex: \t X: \t Y: \n")
for i = 1:Temp.PathSize
    fprintf("%i   \t(%4.3f \t, %4.3f, \t %4.3f) \n",RRTStarTree.Path(i),RRTStarTree.nodes(RRTStarTree.Path(i),1),RRTStarTree.nodes(RRTStarTree.Path(i),2),RRTStarTree.nodes(RRTStarTree.Path(i),3))
end
clear i iteration regenerateCount
figure(2)
    RRTStarTree = RRTStarTree.PlotTree(Grid);
    RRTStarTree = RRTStarTree.PlotPath(Grid);
    legend([RRTStarTree.TreePlot,RRTStarTree.PathPlot],'Tree','Path')
    Temp.iterationString = num2str(Constraints.MaxIterations);
    title("Final Path")
