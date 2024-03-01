classdef Grid
    %GRID Contains the region and grid dimensions with the obstacles
    %   Detailed explanation goes here
    
    properties
        Occupancy = []  ; % Corresponds to indexspace
        indexDimensions = [];
        Resolution
        trueDimensions = [];
    end
    
    methods
        function Grid = Grid(Resolution,Occupancy)
            %GRID Construct an instance of this class
            %   The Grid class conatains the information of the grid occupancy as well as helper functions to provide translation between index space and true space 
            Grid.indexDimensions = size(Occupancy);
            Grid.Resolution = Resolution;
            Grid.trueDimensions = [Grid.indexDimensions(1)*Resolution,Grid.indexDimensions(2)*Resolution];
            Grid.Occupancy = Occupancy;

        end
       
        function Index = getIndex(Grid,point)
           Index = [(point(1) + (1/2)*Grid.Resolution)/Grid.Resolution...
               ,(point(2) + (1/2)*Grid.Resolution)/Grid.Resolution];
        end

        function Point = getPoint(Grid,index)
            Point = [index(1)*Grid.Resolution - (1/2)*Grid.Resolution...
                ,index(2)*Grid.Resolution - (1/2)*Grid.Resolution];
        end

        function check = ContainsObstacle(Grid,gridindex)
            roundedIndex = [round(gridindex(1)),round(gridindex(2))];
            if roundedIndex(1) == 0 || roundedIndex(2) == 0
                check = false;
            elseif roundedIndex(1) >= Grid.indexDimensions(1) || roundedIndex(2) >= Grid.indexDimensions(2)
                check = false;
            elseif 1 == Grid.Occupancy(round(gridindex(1)),round(gridindex(2)))
                check = true;
            else
                check = false;
            end
        end

        function obstacleFree = lineCollisionCheck(Grid,Index1,Index2)
            exit = 0;
            increment = 0.1;
        
            dx = (Index2(1) - Index1(1));
            dy = (Index2(2) - Index1(2));
            r = dx/dy;
            theta = atan(r);
            
            % region check
            if dy < 0 && dx >= 0
                theta = pi + theta;
            elseif dy < 0 && dx <= 0
                theta = theta + pi;
            end
        
            xStep = increment*sin(theta);
            yStep = increment*cos(theta);
        
            lastPoint = Index1;
            tempPoint = [Index1(1) + xStep, Index1(2) + yStep];
            
            if ((Index1(1) - Index2(1)) + (Index1(2) - Index2(2))) < 2*increment
                if (sqrt((Index1(1) - Index2(1))^2 + (Index1(2) - Index2(2))^2)) < increment
                    exit = 1;
                    obstacleFree = 1;
                end
            end

            while exit ~= 1
                
                if 1 == Grid.ContainsObstacle(tempPoint)
                    % hit obstacle, exit
                    obstacleFree = 0;
                    exit = 1;
                elseif sum(abs(tempPoint - Index2)) < increment
                    % reached point
                    obstacleFree = 1;
                    exit = 1;
                end
                lastPoint = tempPoint;
                tempPoint = [tempPoint(1) + xStep, tempPoint(2) + yStep];
            end
        end
    end
end

