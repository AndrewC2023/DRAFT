% Initial draft of Djikstra's algorithm
% September 2023
% Andrew Campbell

% This is the main script



%% Needed Changes and Notes
    % Currently removing visualization, create standalone visualization functions

clc;
clear;
close all;
tic

% Get values from the obstacle Generator
Start = [2,2,2];
End = [29,25,15];

h = 30;
w = 30;
z = 30;

Obstacle_Generator3D;
D = size(Obstacle_MAP);
Max_D = D(1)*D(2)*D(3)*sqrt(2);

Vis = Visualization_Processes3D;
Vis.Start = Start;
Vis.End = End;
Vis = Obstacle_Point_Generation(Vis,Obstacle_MAP,D);
%% A* Loop
% Define Start and End Points

if Start(1) == End(1) && Start(2) == End(2) && Start(3) == End(3)
    error('no path to find')
end

Obstacle_MAP(Start(1),Start(2),Start(3)) = 0; % Start
Obstacle_MAP(End(1),End(2),End(3)) = 0; % End
Visited_Huh = zeros(D(1),D(2),D(3));
Parent_Node = zeros(D(1),D(2),D(3),3);
    % this will give from which point is best to get to the current one 
    % using 1 through 8 with 1 being the right half x ax
    
% For A* we need to pass the G score through and combine it with the H score
Cost_MAP = ones(D(1),D(2),D(3))*Max_D; 
Cost_MAP(Start(1),Start(2),Start(3)) = 0;
G_Score_MAP = Cost_MAP;

% Loop initialization
CP = Start;
% in an assumed grid space this is all the adjacent points
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

while Visited_Huh(End(1),End(2),End(3)) ~=  1
    for n = 1:Neighbor_Num(1)
        Check_Point = CP + adj3D(n,:);
        % Check if the point we are checking is within the array bounds
        if Check_Point(1) == 0 || Check_Point(1) == (D(1) + 1) ...
                || Check_Point(2) == 0 || Check_Point(2) == (D(2) + 1) ...
                || Check_Point(3) == 0 || Check_Point(3) == (D(3) + 1)

            % Point to be Checked is outside Array Bounds
        else
            [Cost,G] = Cost_Function3D(CP,Check_Point,...
                G_Score_MAP(CP(1),CP(2),CP(3)),...
                Obstacle_MAP(Check_Point(1),Check_Point(2),Check_Point(3)), End);
            
            if G_Score_MAP(Check_Point(1),Check_Point(2),Check_Point(3)) > G                    
                Cost_MAP(Check_Point(1),Check_Point(2),Check_Point(3)) = Cost;
                G_Score_MAP(Check_Point(1),Check_Point(2),Check_Point(3)) = G;
                Parent_Node(Check_Point(1),Check_Point(2),Check_Point(3),:) = CP;
            end      
        end
    end   
    
    Visited_Huh(CP(1),CP(2),CP(3)) = 1;
    
    Next_Point = Cost_MAP;
    for i = 1:D(1)
    for j = 1:D(2)   
    for k = 1:D(3)
        if Visited_Huh(i,j,k) == 1
          Next_Point(i,j,k) = Max_D;
        end
    end
    end
    end
    n = 1;
    minC = min(Next_Point(:));
    for i = 1:D(1)
    for j = 1:D(2)   
    for k = 1:D(3)
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

    %% Visualization Tools for path growth.
 
%%
    CP = Potential_Points(next,:);
end
clear r2 CCost MaxD i j r c
hold off

%% Path Generation
    % we work backwards with this method
    CP = End;
    i = 1;
while ((CP(1) ~= Start(1)) + (CP(2) ~= Start(2)) + (CP(3) ~= Start(3))) > 0
%loop start
    Path(i,:) = CP;
    CP = Parent_Node(CP(1),CP(2),CP(3),:);
    i = i + 1;
end
    Path(i,:) = Start;

%%
toc
% Final Visualization Tools
clear i j
Vis_MAP = G_Score_MAP;
for i = 1:D(1)
for j = 1:D(2)
for k = 1:D(3)
    if Visited_Huh(i,j,k) == 0
        Vis_MAP(i,j,k) = NaN;
    end
end
end
end

Vis.Obstacle_Visualization(D,1);
Vis.Path_Visualization(Path,1);


clear i j k adj3D checks Cost CP D G minC n Neighbor_Num next Next_Point ties