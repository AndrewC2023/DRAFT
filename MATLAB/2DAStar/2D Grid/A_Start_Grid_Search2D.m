% Initial draft of Djikstra's algorithm
% September 2023
% Andrew Campbell

% This is the main script

%% Needed Changes and Notes
    % Need to only pass the G score through for each node and then 

clc;
clear;
close all;
Waypoint.Start = [2,2];
Waypoint.End = [30,45];
Obstacle_Generator2D;

tic

% Get values from the obstacle Generator

MAP_Size = size(Obstacle_MAP);
Temp.Max_Distance = MAP_Size(1)*MAP_Size(2)*sqrt(2);

%% A* Loop
% Define Waypoint.Start and Waypoint.End Points

 
Visualization = Visualization_Processes2D;
Visualization.Start = Waypoint.Start;
Visualization.End = Waypoint.End;

if Waypoint.Start(1) == Waypoint.End(1) && Waypoint.Start(2) == Waypoint.End(2)
    error('no path to find')
end

Obstacle_MAP(Waypoint.Start(1),Waypoint.Start(2)) = 0; % Waypoint.Start
Obstacle_MAP(Waypoint.End(1),Waypoint.End(2)) = 0; % Waypoint.End
Visited = zeros(MAP_Size(1),MAP_Size(2));
Parent_Node = zeros(MAP_Size(1),MAP_Size(2),2);
Hueristic_MAP = Visited;
    % this will give from which point is best to get to the current one 
    % using 1 through 8 with 1 being the right half x ax
    
% For A* we need to pass the G score through and combine it with the H score
Cost_MAP = ones(MAP_Size(1),MAP_Size(2))*Temp.Max_Distance; 
Cost_MAP(Waypoint.Start(1),Waypoint.Start(2)) = 0;
G_Score_MAP = Cost_MAP;

% Loop initialization
CP = Waypoint.Start;
% in an assumed grid space this is all the adjacent points
adj2D = [ 1, 0;
         -1, 0;
          1, 1;
         -1,-1;
          0, 1;
          0,-1;
          1,-1;
         -1, 1];
Temp.Neighbor_Num = size(adj2D);
% Check condition will be if the final node has been visited or not
iteration = 1;
while Visited(Waypoint.End(1),Waypoint.End(2)) ~=  1

    % Run a check for each neighbor
    for n = 1:Temp.Neighbor_Num(1)
        Temp.Check_Point = CP + adj2D(n,:);
        % Check if the point we are checking is within the array bounds
        if Temp.Check_Point(1) == 0 || Temp.Check_Point(1) == (MAP_Size(1) + 1) || Temp.Check_Point(2) == 0 || Temp.Check_Point(2) == (MAP_Size(2) + 1)
            % Point to be Checked is outside Array Bounds
        else
            [Cost,Temp.G,Hueristic_MAP(Temp.Check_Point(1),Temp.Check_Point(2))] = Cost_Function2D(CP,Temp.Check_Point,...
                G_Score_MAP(CP(1),CP(2)),...
                Obstacle_MAP(Temp.Check_Point(1),Temp.Check_Point(2)), Waypoint.End);
            
            if G_Score_MAP(Temp.Check_Point(1),Temp.Check_Point(2)) > Temp.G                    
                Cost_MAP(Temp.Check_Point(1),Temp.Check_Point(2)) = Cost;
                G_Score_MAP(Temp.Check_Point(1),Temp.Check_Point(2)) = Temp.G;
                Parent_Node(Temp.Check_Point(1),Temp.Check_Point(2),:) = CP;
            end      
        end
    end   
    Visited(CP(1),CP(2)) = 1;

    %% Visualization Tools for path growth.
    % These can be uncommented to show the grwoth of the path
%     Figure_num = 2;
%     Cost_Map_Visualization(Visualization,G_Score_MAP,MAP_Size,Visited,Figure_num);
%     Path_Visualization(Visualization,CP,Parent_Node,Figure_num);
% 
%     % gif creation
%     if iteration == 1
%         gif('A_Star.gif','overwrite',true)
%     else
%         gif
%     end
%    iteration = iteration + 1;
   
%% Calc next Point   
    Temp.Next_Point = Cost_MAP;
    for i = 1:MAP_Size(1)
        for j = 1:MAP_Size(2)   
          if Visited(i,j) == 1
              Temp.Next_Point(i,j) = Temp.Max_Distance;
          end
        end
    end
    
    % tie breaker
    minC = min(Temp.Next_Point(:));
    [r,c] = find(Temp.Next_Point == minC);
    Temp.ties = size(r);
    if Temp.ties(1) > 1
        for i = 1:Temp.ties(1)
            H_Vals(i,1) =  Hueristic_MAP(r(i),c(i));
            [~, index] = min(H_Vals);      
        end
        Temp.next = index;
        % check if we have a tie here and if so pick a random point now
        Temp.ties2 = size(index);
        if Temp.ties2(1) > 1
            r = randi([1,Temp.ties2(1)]);
            Temp.next = index(r);
        else
        
        end
    else
        Temp.next = 1;
    end
    CP = [r(Temp.next),c(Temp.next)];
    clear H_Vals index 
end
clear Temp i j r c iteration minC

%% Path Generation
    % we work backwards with this method
    CP = Waypoint.End;
    i = 1;
while (CP(1) ~= Waypoint.Start(1)) + (CP(2) ~= Waypoint.Start(2)) > 0
%loop Waypoint.Start
    Path(i,:) = CP;
    CP = Parent_Node(CP(1),CP(2),:);
    i = i + 1;
end 
Path(i,:) = Waypoint.Start;
clear i
toc
% Final Visualization Tools
%     Figure_num = Figure_num + 1;
        Cost_Map_Visualization(Visualization,G_Score_MAP,MAP_Size,Visited,3);
        Path_Visualization(Visualization,Waypoint.End,Parent_Node,3);
figure(4)
    title('Obsacle Map')
    imagesc(-1*Obstacle_MAP)    
    colormap('gray')
    colorbar
    hold on
    plot(Path(:,2),Path(:,1),'g','LineWidth',2.5)
    hold on 
    scatter([Waypoint.Start(2),Waypoint.End(2)],[Waypoint.Start(1),Waypoint.End(1)],[50,50],"filled","square",'g')
    hold off