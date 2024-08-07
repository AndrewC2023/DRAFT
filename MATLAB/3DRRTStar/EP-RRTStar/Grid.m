classdef Grid
    %GRID Contains the region and grid dimensions with the obstacles
    
    properties
        Occupancy = []; % CThe array where each index contains a true false for if the grid is occupied there, this is in index space
        indexDimensions = []; % the number of grid cells on each axis
        Resolution; % the size of the gris cell in true space (meters)
        trueDimensions = []; % the true dimensions of each axis
    end
    
    methods
        function Grid = Grid(Resolution,Occupancy)
            % GRID Constructor
            %   The Grid class conatains the information of the grid occupancy as well as helper functions to provide translation between index space and true space 
            Grid.indexDimensions = size(Occupancy);
            Grid.Resolution = Resolution;
            Grid.trueDimensions = [Grid.indexDimensions(1)*Resolution,Grid.indexDimensions(2)*Resolution,Grid.indexDimensions(3)*Resolution];
            Grid.Occupancy = Occupancy;

        end
        
        % Helper function to convert a point into the corresondind point in the occupancy index space
        function Index = getIndex(Grid,point)
           Index = [(point(1) + (1/2)*Grid.Resolution)/Grid.Resolution,...
                    (point(2) + (1/2)*Grid.Resolution)/Grid.Resolution,...
                    (point(3) + (1/2)*Grid.Resolution)/Grid.Resolution];
        end

        % Helper function to convert an index into a point in true space
        function Point = getPoint(Grid,index)
            Point = [index(1)*Grid.Resolution - (1/2)*Grid.Resolution,...
                     index(2)*Grid.Resolution - (1/2)*Grid.Resolution,...
                     index(3)*Grid.Resolution - (1/2)*Grid.Resolution];
        end

        % function to check if an index is populated by an obstacle
        function check = ContainsObstacle(Grid,gridindex)
            % round to the nearest index as the obstacles extend by 0.5 indexes in all directions and it is easier to directly check the value
            roundedIndex = [round(gridindex(1)),round(gridindex(2)),round(gridindex(3))];
            if roundedIndex(1) == 0 || roundedIndex(2) == 0 || roundedIndex(3) == 0
                % point is out of bounds
                check = false;
            elseif roundedIndex(1) >= Grid.indexDimensions(1) || roundedIndex(2) >= Grid.indexDimensions(2) || roundedIndex(3) >= Grid.indexDimensions(3)
                % point is out of bounds
                check = false;
            elseif 1 == Grid.Occupancy(round(gridindex(1)),round(gridindex(2)),round(gridindex(3)))
                % grid is empty
                check = true;
            else
                % default behavior if an error occurs
                check = false;
            end
        end

        % function to chek if a line intersects an obstacle
        function obstacleFree = lineCollisionCheck(Grid,Index1,Index2)
            % structure is similar to RRT* steer function
            exit = 0;
            increment = 0.1;
            
            vector = Index2 - Index1;
            unit_vector = vector./norm(vector);
    
            xStep = increment*unit_vector(1);
            yStep = increment*unit_vector(2);
            zStep = increment*unit_vector(3);
        
            tempPoint = [Index1(1) + xStep, Index1(2) + yStep, Index1(3) + zStep];
            
            % check if we need to calculate any of this at all, if the two points are super close it is unnecessary
            if norm(Index2 - Index1) < increment
                exit = 1;
                obstacleFree = 1;
            end

            while exit ~= 1
                
                if 1 == Grid.ContainsObstacle(tempPoint)
                    % hit obstacle, exit
                    obstacleFree = 0;
                    exit = 1;
                elseif norm(tempPoint - Index2) < 1.1*increment
                    % reached point
                    obstacleFree = 1;
                    exit = 1;
                end
                tempPoint = [tempPoint(1) + xStep, tempPoint(2) + yStep, tempPoint(3) + zStep];
            end
        end

        % helper to visualize the grid
        function visualizeGrid(Grid, wallsTF, walls)

            if wallsTF == 1
                % for ease plot just walls for speed
                numwalls = size(walls);
                for i = 1:numwalls(1)

                    % const z:
                    fill3([walls(i,1,1), walls(i,1,1), walls(i,1,2), walls(i,1,2)],...
                          [walls(i,2,1), walls(i,2,2), walls(i,2,2), walls(i,2,1)],...
                          [walls(i,3,1), walls(i,3,1), walls(i,3,1), walls(i,3,1)],'black');
                    fill3([walls(i,1,1), walls(i,1,1), walls(i,1,2), walls(i,1,2)],...
                          [walls(i,2,1), walls(i,2,2), walls(i,2,2), walls(i,2,1)],...
                          [walls(i,3,2), walls(i,3,2), walls(i,3,2), walls(i,3,2)],'black');
                    
                    % const y:
                    fill3([walls(i,1,1), walls(i,1,1), walls(i,1,2), walls(i,1,2)],...
                          [walls(i,2,1), walls(i,2,1), walls(i,2,1), walls(i,2,1)],...
                          [walls(i,3,1), walls(i,3,2), walls(i,3,2), walls(i,3,1)],'black');
                    fill3([walls(i,1,1), walls(i,1,1), walls(i,1,2), walls(i,1,2)],...
                          [walls(i,2,2), walls(i,2,2), walls(i,2,2), walls(i,2,2)],...
                          [walls(i,3,1), walls(i,3,2), walls(i,3,2), walls(i,3,1)],'black');
                    
                    % const x:
                    fill3([walls(i,1,1), walls(i,1,1), walls(i,1,1), walls(i,1,1)],...
                          [walls(i,2,1), walls(i,2,1), walls(i,2,2), walls(i,2,2)],...
                          [walls(i,3,1), walls(i,3,2), walls(i,3,2), walls(i,3,1)],'black');
                    fill3([walls(i,1,2), walls(i,1,2), walls(i,1,2), walls(i,1,2)],...
                          [walls(i,2,1), walls(i,2,1), walls(i,2,2), walls(i,2,2)],...
                          [walls(i,3,1), walls(i,3,2), walls(i,3,2), walls(i,3,1)],'black');
                end
                
            else
                for i = 1:Grid.indexDimensions(1)
                for j = 1:Grid.indexDimensions(2)
                for k = 1:Grid.indexDimensions(3)
                    if Grid.Occupancy(i,j,k) == 1
                        scatter3(i,j,k,"black","filled","square");
                        hold on
                    end
                end
                end
                end
            end
        end
    end
end

