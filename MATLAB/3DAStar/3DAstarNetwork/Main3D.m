clc;
clear;
close;

% need to import an image of the rooms that are chosen
ImageImportRooms
Node.Count = 1;
% Generate the nodes more easily based off the image
for i = 1:MAP.Size(1)
for j = 1:MAP.Size(2)
    if MAP.im(i,j,2) < 20 && MAP.im(i,j,3) > 150
    Node.Points(Node.Count,:) = [j,i];
    Node.Count = Node.Count + 1;
    end
end
end
Node.Count = Node.Count - 1;
figure(1)
    scatter(Node.Points(:,1),Node.Points(:,2),Node.Points(:,3))
    hold on
    xlabel('x')
    ylabel('y')
    zlabel('z')

a = size(Node.Connections);
for i = 1:a(1)
figure(1)
    p1 = plot([Node.Points(Node.Connections(i,1),1),Node.Points(Node.Connections(i,2),1)],[Node.Points(Node.Connections(i,1),2),Node.Points(Node.Connections(i,2),2)],'b');
        

    hold on
end
clear i a

Node.Start = 28;
Node.End = 1;

figure(1)
    scatter([Node.Points(Node.Start,1),Node.Points(Node.End,1)],[Node.Points(Node.Start,2),Node.Points(Node.End,2)],'green','filled')
hold on

% Assign all the costs
Node.G_Scores = MAP.Size(1)*MAP.Size(2)*ones(Node.Count,1);
Node.H_Scores = zeros(Node.Count,1);
Node.Costs = MAP.Size(1)*MAP.Size(2)*ones(Node.Count,1);
Node.Visited = zeros(Node.Count,1);
Node.Parent = zeros(Node.Count,1);
Node.Parent(Node.Start) = Node.Start;
Node.Current = Node.Start;
Node.G_Scores(Node.Current) = 0;
% the following is the A Star Algorithm, may want to set it up such that we can call the scrpit on its own

while Node.Visited(Node.End) ~= 1
    [a,b] = find(Node.Connections == Node.Current);
    [numNeighbors,~] = size(a);
    Node.EdgeIndex = [a,b];
    for i = 1:numNeighbors
        if Node.EdgeIndex(i,2) == 1
            Node.EdgeIndex(i,2) = 2;
        elseif Node.EdgeIndex(i,2) == 2
            Node.EdgeIndex(i,2) = 1;
        end
        Node.Neighbors(i) = Node.Connections(Node.EdgeIndex(i,1),Node.EdgeIndex(i,2));
    end
    
    % we now have our neighbor and can start calculating and assigning costs
    for i = 1:numNeighbors
        [Temp.Cost,Temp.G_Score,Node.H_Scores(Node.Current)]....
            = Cost_Function(Node.Points(Node.Current,:)...
                           ,Node.Points(Node.Neighbors(i),:)...
                           ,Node.G_Scores(Node.Current)...
                           ,Node.Points(Node.End,:));

        if Node.G_Scores(Node.Neighbors(i)) > Temp.G_Score
            Node.Costs(Node.Neighbors(i)) = Temp.Cost;
            Node.G_Scores(Node.Neighbors(i)) = Temp.G_Score;
            Node.Parent(Node.Neighbors(i)) = Node.Current;
        end
    end
    Node.Visited(Node.Current) = 1;
    clear a b i
    % Next Point
    a = 1;
    for i = 1:Node.Count
        if Node.Visited(i) == 0 
            Temp.UnvisitedSet(a,:) = [i,Node.Costs(i)];
            a = a + 1;
        end
    end
    Temp.MinCost = min(Temp.UnvisitedSet(:,2));
    [ir,~] = find(Temp.UnvisitedSet(:,2) == Temp.MinCost);
    Temp.Ties = size(ir);
    if Temp.Ties > 1

        % tie break with hueristics     
        for i = 1:Temp.Ties
            Temp.Heuristics(i) = Node.H_Scores(Temp.UnvisitedSet(ir(i),1));
        end
        [~,Temp.HIndex] = min(Temp.Heuristics);
        % Check for double ties
        Temp.ties2 = size(Temp.HIndex);
        if Temp.ties2 > 1
            Temp.r = randi([1,Temp.ties2(1)]);
            Node.Current = Temp.UnvisitedSet(ir(Temp.HIndex(Temp.r)),1);
        else
            Node.Current = Temp.UnvisitedSet(ir(Temp.HIndex(1)),1);
        end

    else
        Node.Current = Temp.UnvisitedSet(ir,1);
    end
   
    clear Temp
    
end


% Path Creation
Path.Current = Node.End;
i = 1;
while Path.Current ~= Node.Start
    Path.Nodes(i) = Path.Current;
    Path.Current = Node.Parent(Path.Current);
    i = i + 1;
end

Path.Nodes(i) = Node.Start;
clear Temp i
a = size(Path.Nodes);
for i = 1:(a(2)-1)
    figure(1)
    p2 = plot([Node.Points(Path.Nodes(i),1),Node.Points(Path.Nodes(i + 1),1)],[Node.Points(Path.Nodes(i),2),Node.Points(Path.Nodes(i + 1),2)],'g','LineWidth',2);
    legend([p1,p2],'Tree','Path')

    hold on
end
hold off
clear a 