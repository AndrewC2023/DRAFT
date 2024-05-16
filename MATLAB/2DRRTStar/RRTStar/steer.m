function [sucessful,newPoint] = steer(Grid,parentPoint,goalPoint,MaxEdgeLength)
    %STEER Summary of this function goes here
    %   Detailed explanation goes here
    
        parentPoint = Grid.getIndex(parentPoint);
        goalPoint = Grid.getIndex(goalPoint);
        sucessful = 0;
        increment = 0.05;
    
        vector = goalPoint - parentPoint;
        unit_vector = vector./norm(vector);
    
    
        xStep = increment*unit_vector(1);
        yStep = increment*unit_vector(2);
    
        lastPoint = parentPoint;
        tempPoint = [parentPoint(1) + xStep, parentPoint(2) + yStep];
        travelled = increment;
    
        if ((parentPoint(1) - goalPoint(1)) + (parentPoint(2) - goalPoint(2))) < 2*increment
            if (sqrt((parentPoint(1) - goalPoint(1))^2 + (parentPoint(2) - goalPoint(2))^2)) < increment
                newPoint = goalPoint;
                sucessful = 1;
            end
        end
        while sucessful ~= 1
            
            if 1 == Grid.ContainsObstacle(tempPoint)
                % hit obstacle, exit
                newPoint = lastPoint;
                sucessful = 1;
            elseif travelled > MaxEdgeLength
                % max length
                newPoint = lastPoint;
                sucessful = 1;
            elseif norm(tempPoint - goalPoint) < increment*1.1
                % reached point
                newPoint = goalPoint;
                sucessful = 1;
            end
            lastPoint = tempPoint;
            tempPoint = [tempPoint(1) + xStep, tempPoint(2) + yStep];
            travelled = travelled + increment;
        end
     
    end
    
    