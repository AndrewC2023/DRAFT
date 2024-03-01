function [Z_Rand] = Random_Node(MAP_Size,Obstacle_MAP)    

    Z_Rounded = [30,10];
    while Obstacle_MAP(Z_Rounded(1),Z_Rounded(2)) == 1
        Z_Rand = rand(1,2);
        Z_Rand(1) = Z_Rand(1)*(MAP_Size(1) - 1) + 1;
        Z_Rand(2) = Z_Rand(2)*(MAP_Size(2) - 1) + 1;
        Z_Rounded = round(Z_Rand);        
    end
end

