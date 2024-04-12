function Cost = Cost(Check_Node,Distance,Obstacles,Cost_0,MAP_Size)
    % Obstacle Proximity
    Check_Points =  [ 1, 0;
                     -1, 0;
                      1, 1;
                     -1,-1;
                      0, 1;
                      0,-1;
                      1,-1;
                     -1, 1;
                      2, 0;
                      2, 1;
                      2,-1;
                      2, 2;
                      2,-2;
                      1, 2;
                      1,-2;
                      0, 2;
                      0,-2;
                     -1, 2;
                     -1,-2;
                     -2, 0;
                     -2, 1;
                     -2,-1;
                     -2, 2;
                     -2,-2];
    
    Rounded = round(Check_Node);
    near_obs_count = 0;
    for i = 1:24
        Check = Rounded + Check_Points(i,:);
        if Check(1) <= 0 || Check(1) >= (MAP_Size(1) + 1) || Check(2) <= 0 || Check(2) >= (MAP_Size(2) + 1)
            % dont check
        else
            if Obstacles(Check(1),Check(2)) == 1
                near_obs_count = near_obs_count + 1;
            end
        end

    end
    
    Cost = Cost_0 + Distance; % + 0*near_obs_count;

end

