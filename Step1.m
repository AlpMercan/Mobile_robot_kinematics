clear;
clc;
origin=[50,50];% base coordinates
dimensions=[5,10];%width and height of robot
diameter=1;%diameter of tyre
v=[1 0]; %linear speed, angular speed
%map
figure
hold on;
axis([0 100 0 100]);
%animation
for i=1:20
    cla;
    origin(2)=origin(2)+v(1);
    %robot Body
    rectangle("Position",[origin(1) origin(2) dimensions(1) dimensions(2)],'FaceColor', 'b')
    %robot tyres
    left_tyre_origin=[origin(1)+dimensions(1) origin(2)+dimensions(2)/2];
    right_tyre_origin=[origin(1) origin(2)+dimensions(2)/2];
    viscircles([left_tyre_origin(1) left_tyre_origin(2)],diameter)
    viscircles([right_tyre_origin(1) right_tyre_origin(2)],diameter)
    %front of the car
    plot(origin(1)+dimensions(1)/2,origin(2)+dimensions(2),'or','MarkerSize',5);
    pause(0.1);
    
end

hold off