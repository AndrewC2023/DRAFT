function [sucessful,newPoint] = steer(Grid,parentPoint,goalPoint,MaxEdgeLength)
%STEER Summary of this function goes here
%   Detailed explanation goes here
% TODO: Fails to work once the sampling region has been made, fix this

    parentPoint = Grid.getIndex(parentPoint);
    goalPoint = Grid.getIndex(goalPoint);
    sucessful = 0;
    increment = 0.05;

    dx = (goalPoint(1) - parentPoint(1));
    dy = (goalPoint(2) - parentPoint(2));
    dz = (goalPoint(3) - parentPoint(3));
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
        elseif sum(abs(tempPoint - goalPoint)) < increment*2
            % reached point
            newPoint = goalPoint;
            sucessful = 1;
        end
        lastPoint = tempPoint;
        tempPoint = [tempPoint(1) + xStep, tempPoint(2) + yStep];
        travelled = travelled + increment;
    end
 
end

