function [sucessful,newPoint] = steer(Grid,parentPoint,goalPoint,MaxEdgeLength)
% steer function effectively extends the tree in the direction of the sampled 
% node until it hits an obstacle, reaches max length, or reaches the node

    parentPoint = Grid.getIndex(parentPoint);
    goalPoint = Grid.getIndex(goalPoint);
    sucessful = 0;
    increment = 0.05;

    vector = goalPoint - parentPoint;
    unit_vector = vector./norm(vector);


    xStep = increment*unit_vector(1);
    yStep = increment*unit_vector(2);

    % loop set up
    lastPoint = parentPoint;
    tempPoint = [parentPoint(1) + xStep, parentPoint(2) + yStep];
    travelled = increment;

    if norm(goalPoint - parentPoint) < increment
        newPoint = goalPoint;
        sucessful = true;
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

