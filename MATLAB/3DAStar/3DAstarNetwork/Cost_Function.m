function [Total_Cost,G,H] = Cost_Function(Current_Point, Check_Point, G_0,End_Point)
    Dist = sqrt((Current_Point(1) - Check_Point(1))^2 + (Current_Point(2) - Check_Point(2))^2 + (Current_Point(3) - Check_Point(3))^2);
    Hueristic = sqrt((Check_Point(1) - End_Point(1))^2 + (Check_Point(2) - End_Point(2))^2 + (Check_Point(3) - End_Point(3))^2);
    % This Term can be modified by a coefficient as well
    G = G_0 + Dist;
    H = 1*Hueristic;
    Total_Cost = G + H;
end