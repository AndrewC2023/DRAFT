clc;
clear;
close;
Start = [2,2,2];
End = [9,3,8];
Obstacle_Generator3D;
Vis = Visualization_Processes3D;

D = size(Obstacle_MAP);

Vis = Obstacle_Point_Generation(Vis,Obstacle_MAP,D);

Vis.Obstacle_Visualization(D)