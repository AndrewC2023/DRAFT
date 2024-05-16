clc;
clear;
close all;


%% Initialize

% Inputs 
Maze;
Constraints.MaxIterations = 3000;
Constraints.MaxEdgeLength = 0.3; % meters
Constraints.GridResolution = 0.05; % meters
Constraints.EndTolerance = 0.05; % meters
Start_End_Indexes = [10,10;
                     85,75];

% Class Construction
Grid = Grid(Constraints.GridResolution,Occupancy'); % transposed due to the nature of how matlab reads Images
clear IndexSize Occupancy Constraints.GridResolution

% create the tree class to  use the helpers in it and to clean up the main loop
RRTStarTree = RRTStarTree(Grid.getPoint([Start_End_Indexes(1,:)]),Grid.getPoint([Start_End_Indexes(2,:)]));
clear Start_End_Indexes

% begin the search loop
foundEnd = false;
iteration = 1;
while iteration <= Constraints.MaxIterations

    
    Temp.newPointCheck = false;
    while Temp.newPointCheck ~= true
        % sample a new point
        Temp.newPoint = RRTStarTree.SampleNewNode(Grid);

        Temp.nearPoint = RRTStarTree.FindNearestNode(Temp.newPoint);
        Temp.nearPoint = RRTStarTree.nodes(Temp.nearPoint,:);

        % Use steer function to "steer" towards the sampled point, 
        % and produce a sampled point that is hno more than the max edge length away from the tree
        [Temp.newPointCheck,newPointIndexPosition] = steer(Grid,Temp.nearPoint,Temp.newPoint,Constraints.MaxEdgeLength/Grid.Resolution);
        
    end
    newPoint = Grid.getPoint(newPointIndexPosition);

    % search for the set of nodes near the new point 
    count = 1;
    for i = 1:RRTStarTree.numNodes
            % this step could use some major optimization
        if ~((1.05*Constraints.MaxEdgeLength < abs(RRTStarTree.nodes(i,1) - newPoint(1))) || (1.05*Constraints.MaxEdgeLength < abs(RRTStarTree.nodes(i,2) - newPoint(2))))

            % 1 means obstacle free
            if 1 == Grid.lineCollisionCheck(Grid.getIndex(RRTStarTree.nodes(i,:)),newPointIndexPosition)
                Temp.nearNodes(count) = i;
                Temp.nearNodeCosts(count) = RRTStarTree.CostFunction(newPoint,i);
                count = count + 1;
            end
        end
    end

    % determine which node should be the parent
    [~,Temp.ind] = min(Temp.nearNodeCosts);
    Temp.ParentNode = Temp.nearNodes(Temp.ind);

    RRTStarTree = RRTStarTree.AddNode(newPoint,Temp.ParentNode);
    % now we check if the new point in the end node

    if newPoint(1) == RRTStarTree.endNode(1) && newPoint(2) == RRTStarTree.endNode(2)
        foundEnd = true;
        RRTStarTree.endNodeIndex = RRTStarTree.numNodes;
    end

    % rewire step
    RRTStarTree = RRTStarTree.Rewire(Temp.nearNodes);
    clear Temp i newPoint newPointIndexPosition count 

    iteration = iteration + 1;

end

if foundEnd == false
    % failed to find a path somehow
    fprintf("Failed to find the path")
    error("");
end

RRTStarTree = RRTStarTree.setPath();
fprintf("Current Path Cost: %f \n",RRTStarTree.costs(RRTStarTree.endNodeIndex))
[~,Temp.PathSize] = size(RRTStarTree.Path);
fprintf("The Final path is: \nIndex: \t X: \t Y: \n")
for i = 1:Temp.PathSize
    fprintf("%i   \t(%4.3f \t, %4.3f) \n",RRTStarTree.Path(i),RRTStarTree.nodes(RRTStarTree.Path(i),1),RRTStarTree.nodes(RRTStarTree.Path(i),2))
end
figure(1)
    RRTStarTree = RRTStarTree.PlotTree(Grid);
    RRTStarTree = RRTStarTree.PlotPath(Grid);
    legend([RRTStarTree.TreePlot,RRTStarTree.PathPlot],'Tree','Path')
    Temp.iterationString = num2str(Constraints.MaxIterations);
    title("Final Path")
