im = imread('Obstacle_Maze.jpg');

% script to load the maxe jpeg file rto test the algorithm,
%  replacing the jpeg with a different one is likely easiest to change the test space

IndexSize = size(im);
IndexSize = [IndexSize(1),IndexSize(2)];
Occupancy = zeros(IndexSize(1),IndexSize(2));

for i = 1:IndexSize(1)
for j = 1:IndexSize(2)
    if im(i,j,1) < 230
    Occupancy(i,j) = 1;
    end
end
end
clear i j im IndexSize

figure(1)
    imagesc(-Occupancy)
    colormap('gray')
    hold on

% Presentation Visualization Section:
    figure(2)
        imagesc(-Occupancy)
        colormap('gray')
        hold on
