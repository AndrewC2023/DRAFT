%% Map Generation
% This is for testing the algorithm of choice with a random set of obstacles

Occupancy = zeros(x,y,z);

% initialize obstacle centers array
numob = 1;

% the obstacles will have a confidence value, ranging from 0 to 256 with 0 being empty
for i = 1:x
for j = 1:y
for k = 1:z
    c = rand;          
    if c < 0.001
        % Randomly general obstacle centers
        Occupancy(i,j,k) = 1;
        
        % Store these locations
        Obstacle_Centers(numob,:) = [i,j,k];
        numob = numob + 1;
    end
end
end
end

% Predefined option
walls = [];
wallsTF = 0;
Predefined = 1;
if Predefined == 1
    clear Obstacle_Centers
    Obstacle_Centers = [];
    % preset random looking ish
    Occupancy = zeros(x,y,z);
    % Obstacle_Centers(1,:) = [15,15,15];
    % Obstacle_Centers(2,:) = [19,15,13];
    % Obstacle_Centers(3,:) = [10,10,10];
    % Obstacle_Centers(4,:) = [12,10,10];
    % Obstacle_Centers(5,:) = [10,12,10];
    % Obstacle_Centers(6,:) = [12,12,12];
    % Obstacle_Centers(7,:) = [10,10,7];
    % Obstacle_Centers(8,:) = [25,25,13];
    % Obstacle_Centers(9,:) = [25,23,13];
    % Obstacle_Centers(10,:) = [25,27,15];
    % Obstacle_Centers(11,:) = [23,25,15];
    % Obstacle_Centers(12,:) = [25,25,9];
    % Obstacle_Centers(13,:) = [25,23,13];
    % Obstacle_Centers(14,:) = [15,13,13];
    % Obstacle_Centers(15,:) = [15,15,11];
    % Obstacle_Centers(16,:) = [13,15,13];
    % Obstacle_Centers(17,:) = [15,13,11];

    % walls:
    clear walls;
    wallsTF = 1;
    walls(1,1,:) = [6,8];
    walls(1,2,:) = [1,20];
    walls(1,3,:) = [1,30];

    walls(2,1,:) = [19,21];
    walls(2,2,:) = [10,30];
    walls(2,3,:) = [1,30];

    walls(3,1,:) = [8,19];
    walls(3,2,:) = [10,20];
    walls(3,3,:) = [13,15];

    sizeWalls = size(walls);
    for n = 1:sizeWalls(1)
        for i = walls(n,1,1):walls(n,1,2)
        for j = walls(n,2,1):walls(n,2,2)
        for k = walls(n,3,1):walls(n,3,2)
            Obstacle_Centers = [Obstacle_Centers; i,j,k];
        end
        end
        end
    end
    clear sizeWalls
    a = size(Obstacle_Centers);
    for i = 1:a(1)
        Occupancy(Obstacle_Centers(i,1),Obstacle_Centers(i,2),Obstacle_Centers(i,3)) = 1;
    end
end

numob = numob - 1;
if numob == 0
    error('No obstacles generated')
end
% Obstacle Expansion
checks = size(Obstacle_Centers);
if wallsTF ~= 1
    for i = 1:x
    for j = 1:y
    for k = 1:z
        % this is based upon distance
        % and for general form we can check all obstacles
        for q = 1:checks(1)
            dist2ob = sqrt((i - Obstacle_Centers(q,1))^2 + (j - Obstacle_Centers(q,2))^2 + (k - Obstacle_Centers(q,3))^2);
            if dist2ob < 1.9
                Occupancy(i,j,k) = Occupancy(Obstacle_Centers(q,1),Obstacle_Centers(q,2),Obstacle_Centers(q,3));
            end
            % Use this oppourtunity to make sure the starts and ends are clear
            dist2start = sqrt((Start_End_Indexes(1,1) - i)^2 + (Start_End_Indexes(1,2) - j)^2 + (Start_End_Indexes(1,3) - k)^2);
            if dist2start < 3
                Occupancy(i,j,k) = 0;
            end
            dist2end = sqrt((Start_End_Indexes(2,1) - i)^2 + (Start_End_Indexes(2,2) - j)^2 + (Start_End_Indexes(2,3) - k)^2);
            if dist2end < 3
                Occupancy(i,j,k) = 0;
            end
        end
    end
    end
    end
end

clear c i j Obstacle_Centers k q numob cheks dist2ob Predefined dist2start dist2end