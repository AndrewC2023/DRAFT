function [sucessful,newPoint] = steer(Grid,parentPoint,goalPoint,MaxEdgeLength)
% steer function takes the nearest node to the point and extends an edge in that direction
% until it hits an obstacle, reaches the node, or reaches the maximum edge length

    parentPoint = Grid.getIndex(parentPoint);
    goalPoint = Grid.getIndex(goalPoint);
    sucessful = false;
    increment = 0.05;

    % get the vector that we move along
    vector = goalPoint - parentPoint;
    unit_vector = vector./norm(vector);

    xStep = increment*unit_vector(1);
    yStep = increment*unit_vector(2);
    zStep = increment*unit_vector(3);

    % loop set up
    lastPoint = parentPoint;
    tempPoint = [parentPoint(1) + xStep, parentPoint(2) + yStep, parentPoint(3) + zStep];
    travelled = increment;

    if norm(goalPoint - parentPoint) < increment
        newPoint = goalPoint;
        sucessful = true;
    end

    while sucessful ~= true
        
        if 1 == Grid.ContainsObstacle(tempPoint)
            % hit obstacle, exit
            newPoint = lastPoint;
            sucessful = true;
        elseif travelled > MaxEdgeLength
            % max length reached
            newPoint = lastPoint;
            sucessful = true;
        elseif norm(tempPoint - goalPoint) < increment*1.1
            % reached point
            newPoint = goalPoint;
            sucessful = true;
        end
        lastPoint = tempPoint;
        tempPoint = [tempPoint(1) + xStep, tempPoint(2) + yStep, tempPoint(3) + zStep];
        travelled = travelled + increment;
    end
 
end

