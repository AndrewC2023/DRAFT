clc;
clear;
close;


%% Initialization
% Generate Obstacles to navigate around
%      Waypoint.Start = [2,2];
%      Waypoint.End = [9,10];
%      Obstacle_GeneratorRRT2D;
%      MAP_Size = size(Obstacle_MAP);
%      MAP_Size = [MAP_Size(1),MAP_Size(2)];


Maze;
tic
Waypoint.Start = [10,10];
Waypoint.End = [75,85];
MAX_ITERATIONS = 2000;
Max_Distance = MAP_Size(1)*MAP_Size(2)*sqrt(2);
% Initialize classes and create the Start Node

Node = NodeXY;
Node.Nodes(1,:) = Waypoint.Start; % Start node is Node 1
Node.Num_Nodes = size(Node.Nodes);
Edges = Edge;
Edges.Max_Edge_Length = 3;
Node.Node_Cost(1,1) = 0;
Steer_Step_Size = 0.1;
iteration = 2;
while iteration < MAX_ITERATIONS
    % Random obstacle free node generation
    Position.New = 0;
    Manhattan_Distances = zeros(Node.Num_Nodes(1),1);
    while Position.New == 0
        Position.Random = Random_Node(MAP_Size,Obstacle_MAP);

        for i = 1:Node.Num_Nodes(1)
            Manhattan_Distances(i) = abs(Node.Nodes(i,1) - Position.Random(1))...
                + abs(Node.Nodes(i,2) - Position.Random(2));
        end
        [~,Nearest_Node] = min(Manhattan_Distances);
        clear i 
        Position.New = Steer(Position.Random,Node.Nodes(Nearest_Node,:),Steer_Step_Size,Obstacle_MAP,Edges.Max_Edge_Length);
    end

    % create the set of nodes closest to the new node
    n = 1;
    for i = 1:Node.Num_Nodes(1)
        if (abs(Node.Nodes(i,1) - Position.New(1)) <= Edges.Max_Edge_Length * 1.1) + (abs(Node.Nodes(i,2) - Position.New(2)) <= Edges.Max_Edge_Length*1.1) == 2
            % check if we can drive to this node
            Drivable = DrawCollisionCheck(Node.Nodes(i,:),Position.New,0.2,Obstacle_MAP);
            True_Distance = sqrt((Node.Nodes(i,1) - Position.New(1))^2 + (Node.Nodes(i,2) - Position.New(2))^2);
            if ((True_Distance <= Edges.Max_Edge_Length) && (Drivable == 1))
                Potential_Cost = Cost(Position.New,True_Distance,Obstacle_MAP,Node.Node_Cost(i),MAP_Size);                
                Near_Nodes(n,:) = [True_Distance,Potential_Cost,i];
                n = n + 1;
            end
        end
    end
    n = n - 1;
    % Find the parent node    
    [~,which_neighbor] = min(Near_Nodes(:,2));
    Parent_Node = Near_Nodes(which_neighbor,3);
    
    % Define our new node
    Node = Node.NodeGeneration(Position.New); 

    
    Node.Node_Cost(iteration) = Near_Nodes(which_neighbor,2);

    % generate this new edge
    Edges = Edges.Create_New_Edge(Parent_Node,iteration,True_Distance);
    
    % Rewire function
    temp = 1;
    for i = 1:n
        if i ~= which_neighbor
        Near_Nodes_No_Parent(temp,:) = Near_Nodes(i,:);
        
        temp = temp + 1;
        end
    end
    temp = size(Near_Nodes);
    if temp(1) <= 1
        % skip
    else
       Edges = Edges.Rewire(Near_Nodes_No_Parent(:,3),Near_Nodes(which_neighbor,2),Node,Obstacle_MAP,MAP_Size,iteration);    
    end

    % Rewire the rewires
    clear Near_Nodes which_neighbor temp Near_Nodes_No_Parent Potential_Cost n
    
    iteration = iteration + 1;  
end
toc
figure(1)
p1 = Tree_Visualization(Node.Nodes,Edges.Edges(:,[1,2]));

%% Path generation

% Create the final node
Node = Node.NodeGeneration(Waypoint.End);

% Find the nearest node to that and conncet them
for i = 1:Node.Num_Nodes(1) - 1
    Manhattan_Distances(i) = abs(Node.Nodes(i,1) - Node.Nodes(MAX_ITERATIONS,(1)))...
        + abs(Node.Nodes(i,2) - Node.Nodes(MAX_ITERATIONS,(2)));
end
[~,Nearest_Node] = min(Manhattan_Distances);
Edges = Edges.Create_New_Edge(Nearest_Node,iteration,0);

Current_Path_Node = iteration;
i = 1;
while Current_Path_Node ~= 1
    % serialize the coordinates
    x(i) = Node.Nodes(Current_Path_Node,2);
    y(i) = Node.Nodes(Current_Path_Node,1);

    ind = find(Edges.Edges(:,2) == Current_Path_Node);
    Current_Path_Node = Edges.Edges(ind,1);
    i = i + 1;
end

x(i) = Node.Nodes(Current_Path_Node,2);
y(i) = Node.Nodes(Current_Path_Node,1);

figure(1)
    p2 = plot(x,y,'g','LineWidth',2);
    legend([p1,p2],'Tree','Path')

clear iteration i ind x y Current_Path_Node temp Manhattan_Distances Drivable Current_Path_Node Position True_Distance





