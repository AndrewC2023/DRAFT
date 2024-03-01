function Z_New = Steer(x_target,x_nearest,Steer_Step,Obstacle_MAP,Max_Edge_Length)
    di = (x_target(1) - x_nearest(1));
    dj = (x_target(2) - x_nearest(2));
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

    Dist_Travelled = Steer_Step;
    
    Temp_Point = x_nearest;
    exit = 0;

    while exit ~= 1       
        Temp_Point_Rounded = round(Temp_Point);
        if Obstacle_MAP(Temp_Point_Rounded(1),Temp_Point_Rounded(2)) == 1
            % hit an obstacle, exit
            Z_New = Last_Point;
            exit = 1;
        elseif sum(abs(Temp_Point - x_target)) < Steer_Step
            % no obstacles found between the two, reached target
            Z_New = x_target;
            exit = 1;
        elseif Dist_Travelled > Max_Edge_Length - 1
            % farthest edge length willing to create reached
            Z_New = Last_Point;
            exit = 1;   
        end
        Last_Point = Temp_Point;
        Temp_Point = [Last_Point(1) + i_step,Last_Point(2) + j_step];
        Dist_Travelled = Dist_Travelled + Steer_Step;
    end

    if Z_New == x_nearest
        % created a repeat node, not worth
        Z_New = 0;
    end

end