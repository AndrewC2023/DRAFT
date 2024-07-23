clc;
clear;
close all;


%% Initialize

% Problem setup
Constraints.MaxIterations = 5000;
Constraints.MaxEdgeLength = 0.08; % meters
Constraints.GridResolution = 0.05; % meters
Constraints.EndTolerance = 0.05; % meters
Start_End_Indexes = [2,2,2;
                     29,25,15]; % start and end points
x = 30;
y = 30;
z = 30;

Obstacle_Generator3D; % obstacle generator

% Class Construction
Grid = Grid(Constraints.GridResolution,Occupancy);
clear IndexSize Occupancy Constraints.GridResolution

RRTStarTree = RRTStarTree(Grid.getPoint([Start_End_Indexes(1,:)]), Grid.getPoint([Start_End_Indexes(2,:)]));
clear Start_End_Indexes

% begin the search loop
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
        if ~((1.05*Constraints.MaxEdgeLength < abs(RRTStarTree.nodes(i,1) - newPoint(1))) || (1.05*Constraints.MaxEdgeLength < abs(RRTStarTree.nodes(i,2) - newPoint(2))) || (1.05*Constraints.MaxEdgeLength < abs(RRTStarTree.nodes(i,3) - newPoint(3))))

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

    if newPoint(1) == RRTStarTree.endNode(1) && newPoint(2) == RRTStarTree.endNode(2) && newPoint(3) == RRTStarTree.endNode(3)
        RRTStarTree.endNodeIndex = RRTStarTree.numNodes;
    end

    % rewire step
    RRTStarTree = RRTStarTree.Rewire(Temp.nearNodes);
    clear Temp i newPoint newPointIndexPosition count 

    iteration = iteration + 1;

end

RRTStarTree = RRTStarTree.setPath();
fprintf("Path Cost: %f \n",RRTStarTree.costs(RRTStarTree.endNodeIndex))
[~,Temp.PathSize] = size(RRTStarTree.Path);
fprintf("The Final path is: \nIndex: \t X: \t Y: \n")
for i = 1:Temp.PathSize
    fprintf("%i   \t(%4.3f \t, %4.3f, \t %4.3f) \n",RRTStarTree.Path(i),RRTStarTree.nodes(RRTStarTree.Path(i),1),RRTStarTree.nodes(RRTStarTree.Path(i),2),RRTStarTree.nodes(RRTStarTree.Path(i),3))
end
clear i regenerateCount
figure(1)
    % RRTStarTree = RRTStarTree.PlotTree(Grid);
    RRTStarTree = RRTStarTree.PlotPath(Grid);
    Grid.visualizeGrid();
    legend([RRTStarTree.TreePlot,RRTStarTree.PathPlot],'Tree','Path') % warning but not error displays if both plot tree and plot path are not uncommented
    Temp.iterationString = num2str(Constraints.MaxIterations);
    title("Final Path")
