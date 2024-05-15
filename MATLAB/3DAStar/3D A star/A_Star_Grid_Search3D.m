
% September 2023
% Andrew Campbell

% This is the main script

%% Needed Changes and Notes
    % Currently removing visualization, create standalone visualization functions

clc;
clear;
close all;
tic

% set up the problem and generate some obstacles, or load predefined obstacles
% Define Start and End Points
Nodes.Start = [2,2,2];
Nodes.End = [29,25,15];

% configuration space dimensions
h = 30;
w = 30;
z = 30;

Obstacle_Generator3D;

Grid.Dimensions  = size(Obstacle_MAP);
Grid.Occupancy = Obstacle_MAP;
clear Obstacle_MAP
Grid.Max_Distance = Grid.Dimensions(1)*Grid.Dimensions(2)*Grid.Dimensions(3)*sqrt(2);

%% A* Loop

% sanity check
if Nodes.Start(1) == Nodes.End(1) && Nodes.Start(2) == Nodes.End(2) && Nodes.Start(3) == Nodes.End(3)
    error('no path to find')
end

Visited = zeros(Grid.Dimensions(1),Grid.Dimensions(2),Grid.Dimensions(3));
Nodes.Parent_Node = zeros(Grid.Dimensions(1),Grid.Dimensions(2),Grid.Dimensions(3),3);
    % this will give from which point is best to get to the current one 
    % using 1 through 8 with 1 being the right half x ax
    
% For A* we need to pass the G score through and combine it with the H score
Nodes.Cost_MAP = ones(Grid.Dimensions(1),Grid.Dimensions(2),Grid.Dimensions(3))*Grid.Max_Distance; 
Nodes.Cost_MAP(Nodes.Start(1),Nodes.Start(2),Nodes.Start(3)) = 0;
Nodes.G_Scores = Nodes.Cost_MAP;

% Loop initialization
Current_Node = Nodes.Start;
% in an assumed grid space this is all the adjacent points, it is messy but faster to predefone all these points
adj3D = [ 1, 0, 0;
          0, 1, 0;
          0, 0, 1;
         -1, 0, 0;
          0,-1, 0;
          0, 0,-1;
          1, 1, 0;
         -1, 1, 0;
          1,-1, 0;
         -1,-1, 0;
          0, 1, 1;
          0,-1, 1;
          0, 1,-1;
          0,-1,-1;
          1, 0, 1;
         -1, 0, 1;
          1, 0,-1;
         -1, 0,-1;
          1, 1, 1;
         -1,-1,-1;
         -1, 1, 1;
          1,-1,-1;
          1,-1, 1;
         -1, 1,-1;
          1, 1,-1;
         -1, 1, 1];
Neighbor_Num = size(adj3D);      
% Check condition will be if the final node has been visited or not

while Visited(Nodes.End(1),Nodes.End(2),Nodes.End(3)) ~=  1
    for n = 1:Neighbor_Num(1)
        Neighbor_Node = Current_Node + adj3D(n,:);
        % Check if the point we are checking is within the array bounds
        if Neighbor_Node(1) == 0 || Neighbor_Node(1) == (Grid.Dimensions(1) + 1) ...
                || Neighbor_Node(2) == 0 || Neighbor_Node(2) == (Grid.Dimensions(2) + 1) ...
                || Neighbor_Node(3) == 0 || Neighbor_Node(3) == (Grid.Dimensions(3) + 1)

            % Point to be Checked is outside Array Bounds, do nothing and continue
        else
            % check if the point can be given the current point as a parent
            [Cost,G] = Cost_Function3D(Current_Node,Neighbor_Node,...
                Nodes.G_Scores(Current_Node(1),Current_Node(2),Current_Node(3)),...
                Grid.Occupancy(Neighbor_Node(1),Neighbor_Node(2),Neighbor_Node(3)), Nodes.End);

            % compare costs
            if Nodes.G_Scores(Neighbor_Node(1),Neighbor_Node(2),Neighbor_Node(3)) > G                    
                Nodes.Cost_MAP(Neighbor_Node(1),Neighbor_Node(2),Neighbor_Node(3)) = Cost;
                Nodes.G_Scores(Neighbor_Node(1),Neighbor_Node(2),Neighbor_Node(3)) = G;
                Nodes.Parent_Node(Neighbor_Node(1),Neighbor_Node(2),Neighbor_Node(3),:) = Current_Node;
            end      
        end
    end   
    
    % Mark the current Node as visited, so that we no longer use it as a current node
    Visited(Current_Node(1),Current_Node(2),Current_Node(3)) = 1;
    
    Next_Point = Nodes.Cost_MAP;
    for i = 1:Grid.Dimensions(1)
    for j = 1:Grid.Dimensions(2)   
    for k = 1:Grid.Dimensions(3)
        if Visited(i,j,k) == 1
          Next_Point(i,j,k) = Grid.Max_Distance;
        end
    end
    end
    end

    n = 1;
    minC = min(Next_Point(:));
    for i = 1:Grid.Dimensions(1)
    for j = 1:Grid.Dimensions(2)   
    for k = 1:Grid.Dimensions(3)
        if Next_Point(i,j,k) == minC
          Potential_Points(n,:) = [i,j,k];
          n = n + 1;
        end
    end
    end
    end

    ties = size(Potential_Points);
    if ties(1) > 1
        % Revamp for the one with the Lowest H value, if that is the same, make it random
        next = randi([1,ties(1)]);
    else
        next = 1;
    end

    Current_Node = Potential_Points(next,:);
end
clear r2 CCost MaxD i j r c
hold off

%% Path Generation
    % we work backwards with this method
    Current_Node = Nodes.End;
    i = 1;
while ((Current_Node(1) ~= Nodes.Start(1)) + (Current_Node(2) ~= Nodes.Start(2)) + (Current_Node(3) ~= Nodes.Start(3))) > 0
%loop start
    Path(i,:) = Current_Node;
    Current_Node = Nodes.Parent_Node(Current_Node(1),Current_Node(2),Current_Node(3),:);
    i = i + 1;
end
    Path(i,:) = Nodes.Start;

%%
toc

clear i j

% Visualization Tools, not very optimized
Vis = Visualization;
Vis.Start = Nodes.Start;
Vis.End = Nodes.End;
Vis = Obstacle_Point_Generation(Vis,Grid.Occupancy,Grid.Dimensions);

Vis.Obstacle_Visualization(Grid.Dimensions,1);
Vis.Path_Visualization(Path,1);


clear i j k adj3D checks Cost Current_Node G minC n Neighbor_Num next Next_Point ties Potential_Points