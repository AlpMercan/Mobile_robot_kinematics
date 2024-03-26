clear;
clc;
%% Assumptions
%turtlebot3 burgers dimensions will be used
%the robot will look like a box with two wheels on each side and those are located in the back end of the robot
%no sliding
%differential tyre lay out
%flat and equal terrain
%wheels are fixed, no holonomic actions
%% Map
origin = [500 500];
figure;
hold on;
axis([0 1000 0 1000]);
%% Robots Dimensions
robot_size = [138 178 192]; % L x W x H in mm
Tyre_diameter = 66; % mm
Tyre_Width = 20; % mm for visualization
Tyre_distance = 160; % mm distance between tyres
%% Max Speeds
v_max = 0.22; %mm/s max translational velocity
w_max = 2.84; %rad/s max angular velocity
%% Kinematics Equations
theta = 0; % initial orientation give in rad
vr = 0.1; % right tyre velocity mm/s
vl = -0.1; % left tyre velocity mm/s
R = Tyre_diameter/2;
L = Tyre_distance/2;
tyre_matrix = [vr ; vl];
middle_matrix = [R*cos(theta)/2 , R*cos(theta)/2 ; R*sin(theta)/2, R*sin(theta)/2 ; R/(2*L), -R/(2*L) ];
outcome_matrix= middle_matrix*tyre_matrix;
vx=outcome_matrix(1);
vy=outcome_matrix(2);
w=outcome_matrix(3);
%animation
dt=1;%for instantenous movement
number_of_iterations=50;%%length of duration
for i=1:number_of_iterations
    %because of the orientation the rectangle function in step 1 can not be
    %used
    cla;
    origin(1)=origin(1)+vx*dt;% Update position
    origin(2)=origin(2)+vy*dt;% Update position
    theta = theta + w*dt; % Update orientation
    x = [origin(1)-robot_size(1)/2, origin(1)+robot_size(1)/2, origin(1)+robot_size(1)/2, origin(1)-robot_size(1)/2, origin(1)-robot_size(1)/2];
    y = [origin(2)-robot_size(2)/2, origin(2)-robot_size(2)/2, origin(2)+robot_size(2)/2, origin(2)+robot_size(2)/2, origin(2)-robot_size(2)/2];
    % Rotate rectangle around its center
    rotation_matrix = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    rotated_vertices = rotation_matrix * [x - mean(x); y - mean(y)];
    rotated_x = rotated_vertices(1,:) + mean(x);
    rotated_y = rotated_vertices(2,:) + mean(y);
    % Plot rotated rectangle
    patch(rotated_x, rotated_y, 'b', 'EdgeColor', 'b', 'FaceAlpha', 0.3);
    % Calculate tire positions relative to rectangle's center
    left_tyre_origin = [-robot_size(1)/2+14, 18+Tyre_distance/2 ];
    right_tyre_origin = [-robot_size(1)/2+14, 18-(Tyre_distance/2 )];
    % Rotate tire positions
    rotated_left_tyre_origin = rotation_matrix * left_tyre_origin' + [mean(x); mean(y)];
    rotated_right_tyre_origin = rotation_matrix * right_tyre_origin' + [mean(x); mean(y)];
    % Plot tires
    viscircles([rotated_left_tyre_origin(1), rotated_left_tyre_origin(2)], Tyre_Width)
    viscircles([rotated_right_tyre_origin(1), rotated_right_tyre_origin(2)], Tyre_Width)
    % Plot front circle (front of the car)
    front_circle = rotation_matrix * [robot_size(1)/2; 18] + [mean(x); mean(y)];
    plot(front_circle(1), front_circle(2), 'or', 'MarkerSize', 5);
    pause(0.1);
end
hold off
