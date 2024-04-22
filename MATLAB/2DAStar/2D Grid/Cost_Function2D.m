function [Cost,G,H] = Cost_Function2D(Current_Point, Check_Point, G_0, Obstacle_Val,End_Point)
    Dist = sqrt((Current_Point(1) - Check_Point(1))^2 + (Current_Point(2) - Check_Point(2))^2);
    Hueristic = sqrt((Check_Point(1) - End_Point(1))^2 + (Check_Point(2) - End_Point(2))^2);
    % This Term can be modified by a coefficient as well
    G = G_0 + (Dist + (20*Obstacle_Val)/256);
    H = 1*Hueristic;
    Cost = G + H;
end