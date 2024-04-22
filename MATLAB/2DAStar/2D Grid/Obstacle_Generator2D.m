%% Map Generation
% This is for testing the algorithm of choice
h = 45;
w = 45;
Obstacle_MAP = zeros(h,w);
numob = 1;

% the obstacles will have a confidence value, ranging from 0 to 256 with 0 being empty
for i = 1:h
    for j = 1:w
        c = rand;          
        if c < 0.01
            % Randomly general obstacle centers
            Obstacle_MAP(i,j) = 256;
            
            % Store these locations
            Obstacle_Centers(numob,:) = [i,j];
            numob = numob + 1;
        end

    end
end

% Obstacle Expansion
checks = size(Obstacle_Centers);
for i = 1:h
    for j = 1:w
        % this is based upon distance
        % and for general form we can check all obstacles
        for q = 1:checks(1)
            dist2ob = sqrt((i - Obstacle_Centers(q,1))^2 + (j - Obstacle_Centers(q,2))^2);
            if dist2ob < 2.9
                Obstacle_MAP(i,j) = Obstacle_MAP(Obstacle_Centers(q,1),Obstacle_Centers(q,2));
            end
        end
        distS = sqrt((Waypoint.Start(1) - i)^2 + (Waypoint.Start(2) - j)^2);
        distE = sqrt((Waypoint.End(1) - i)^2 + (Waypoint.End(2) - j)^2);

        if distS < 3 || distE < 3
            Obstacle_MAP(i,j) = 0;
        end

    end
end
% This generalization can be expanded to 3D
clear c i j h w checks dist2ob numob Obstacle_Centers q distE distS
Figure_Num = 1; 
figure(Figure_Num)
    title('Obsacle Map')
    imagesc(-1*Obstacle_MAP)    
    colormap('gray')

Figure_Num = Figure_Num + 1; 