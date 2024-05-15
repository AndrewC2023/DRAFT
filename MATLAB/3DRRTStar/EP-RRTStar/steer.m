function [sucessful,newPoint] = steer(Grid,parentPoint,goalPoint,MaxEdgeLength)
%STEER Summary of this function goes here
%   Detailed explanation goes here
% TODO: Fails to work once the sampling region has been made, fix this

    parentPoint = Grid.getIndex(parentPoint);
    goalPoint = Grid.getIndex(goalPoint);
    sucessful = 0;
    increment = 0.05;

    vector = goalPoint - parentPoint;
    unit_vector = [vector(1)/norm(vector), vector(2)/norm(vector), vector(3)/norm(vector)];


    xStep = increment*unit_vector(1);
    yStep = increment*unit_vector(2);
    zStep = increment*unit_vector(3);

    lastPoint = parentPoint;
    tempPoint = [parentPoint(1) + xStep, parentPoint(2) + yStep, parentPoint(3) + zStep];
    travelled = increment;


    if norm(goalPoint - parentPoint) < increment
        newPoint = goalPoint;
        sucessful = 1;
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
        tempPoint = [tempPoint(1) + xStep, tempPoint(2) + yStep, tempPoint(3) + zStep];
        travelled = travelled + increment;
    end
 
end

