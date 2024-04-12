function No_Obstacles = DrawCollisionCheck(x_target,x_new,Steer_Step,Obstacle_MAP)
    exit = 0;
    di = (x_target(1) - x_new(1));
    dj = (x_target(2) - x_new(2));
    if di == 0 || dj == 0
        exit = 1;
        No_Obstacles = 1;
    end
    r = di/dj;
    theta = atan(r);
    % region check
    if dj < 0 && di >= 0
        theta = pi + theta;
    elseif dj < 0 && di <= 0
        theta = theta + pi;
    end

    i_step = Steer_Step*sin(theta);
    j_step = Steer_Step*cos(theta);
    
    Temp_Point = x_new;
    while exit ~= 1
        
        Temp_Point_Rounded = round(Temp_Point);
        if Obstacle_MAP(Temp_Point_Rounded(1),Temp_Point_Rounded(2)) == 1
            % hit an obstacle, exit
            No_Obstacles = 0;
            exit = 1;
        elseif sum(abs(Temp_Point - x_target)) < Steer_Step
            % no obstacles found between the two, reached target
            No_Obstacles = 1;
            exit = 1;
        end      
        Last_Point = Temp_Point;
        Temp_Point = [Last_Point(1) + i_step,Last_Point(2) + j_step];
    end

end