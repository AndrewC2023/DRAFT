im = imread('Obstacle_Maze.jpg');

MAP_Size = size(im);
MAP_Size = [MAP_Size(1),MAP_Size(2)];
Obstacle_MAP = zeros(MAP_Size(1),MAP_Size(2));

for i = 1:MAP_Size(1)
for j = 1:MAP_Size(2)
    if im(i,j,1) < 230
    Obstacle_MAP(i,j) = 1;
    end
end
end
clear i j im



figure(1)
    imagesc(-Obstacle_MAP)
    colormap('gray')
    hold on