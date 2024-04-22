MAP.im = imread('RoomExample.png','png');

MAP.Size = size(MAP.im);

figure(1)
    image(MAP.im)
    hold on