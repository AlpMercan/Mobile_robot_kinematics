clear;
clc;
%% Assumptions
% turtlebot3 burgers dimensions will be used
% the robot will look like a box with two wheels on each side and those are located in the back end of the robot
% no sliding
% differential tyre lay out
% flat and equal terrain
% wheels are fixed, no holonomic actions
%% Map
origin = [500 500]; % starting point [x,y,theta]
path = [1400 1400];
figure;
hold on;
axis([-1000 2000 -1000 2000]);
%% Robots Dimensions
robot_size = [178 138 192]; % L x W x H in mm
Tyre_diameter = 66; % mm
Tyre_Width = 20; % mm for visualization
Tyre_distance = 160; % mm distance between tyres
%% Max Speeds
v_max = 0.22; %mm/s max translational velocity
w_max = 2.84; %rad/s max angular velocity
%% Polar convertion
R = Tyre_diameter / 2;
L = Tyre_distance / 2;
theta_initial = 0;%for visualizising first
theta = 0; % initial orientation to reference frame
delta_x = path(1) - origin(1);
delta_y = path(2) - origin(2);
p = sqrt(delta_x^2 + delta_y^2) + 0.01; % 0.01 for prevent p from becoming 0
alfa = atan2(delta_y, delta_x) - theta; % Updated initial orientation to path
beta = -theta - alfa; % Updated final orientation

%% Main loop
while p > 0.1 
    delta_x = path(1) - origin(1);
    delta_y = path(2) - origin(2);

    p = sqrt(delta_x^2 + delta_y^2) + 0.01; % 0.01 for prevent p from becoming 0
    alfa = +(atan2(delta_y, delta_x) - theta); % Updated initial orientation to path
    beta = -theta - alfa; % Updated final orientation

    %% the control law
    kp = 0.1;
    k_alfa = 0.8;
    k_beta = -0.1;
    v = kp * p;
    w = k_alfa * alfa + k_beta * beta;
    transform_matrix = [cos(alfa), 0; -(sin(alfa)) / p, -1; (sin(alfa)) / p, 0];
    speed_matrix = [v; w];
    output_matrix = transform_matrix * speed_matrix;
    vp = output_matrix(1);
    v_alfa = output_matrix(2);
    v_beta = output_matrix(3);

    % Update position and orientation
    origin(1) = origin(1) + vp * cos(beta);
    origin(2) = origin(2) - vp * sin(beta);
    beta = -theta - v_alfa;

    % Visualization
    clf

    hold on;
    axis([0 2000 0 2000]);

    % Draw robot
    draw_robot(500, 500, theta_initial, robot_size, L, Tyre_distance, 0);
    draw_robot(origin(1), origin(2), beta, robot_size, L, Tyre_distance, Tyre_Width);

    % Plot path
    plot([origin(1) path(1)],[origin(2) path(2)],"g")
    plot(origin(1), origin(2), 'ro'); % Robot position
    plot(path(1), path(2), 'gx'); % Target position
    drawnow; % Update figure
    pause(0.1)
end
function draw_robot(x, y, theta, size, L, Tyre_distance, Tyre_Width)
    % Define robot shape
    robot_shape = [
        -size(2)/2, size(2)/2, size(2)/2, -size(2)/2, -size(2)/2; % x coordinates
        -size(1)/2, -size(1)/2, size(1)/2, size(1)/2, -size(1)/2  % y coordinates
    ];

    % Rotate and translate robot shape
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    robot_shape = R * robot_shape;
    robot_shape(1, :) = robot_shape(1, :) + x;
    robot_shape(2, :) = robot_shape(2, :) + y;

    % Draw robot body (rectangle)
    fill(robot_shape(1, :), robot_shape(2, :), 'b');

    % Plot circles (tires) at the back of the robot
    back_left_tire_y = y +(size(1) / 2 ) * cos(theta) - Tyre_distance*cos(theta)*0.5;
    back_right_tire_y = y - (size(1) / 2 ) * sin(theta) -Tyre_distance*cos(theta)*0.5;
    back_left_tire_x = x - (size(2) / 2 ) * cos(theta) - Tyre_distance/2; % Adjusted position for the left tire
    back_right_tire_x = x - (size(2) / 2 ) * cos(theta) + Tyre_distance/2; % Adjusted position for the right tire
    rotated_back_left_tire = [back_left_tire_x; back_left_tire_y];
    rotated_back_right_tire = [back_right_tire_x; back_right_tire_y];
    viscircles(rotated_back_left_tire', Tyre_Width / 2, 'EdgeColor', 'r');
    viscircles(rotated_back_right_tire', Tyre_Width / 2, 'EdgeColor', 'r');

    % Plot marker (front of the robot)
    marker_x = x + (size(2) / 2 ) * cos(theta);
    marker_y = y + (size(1) / 2 ) * sin(theta);
    plot(marker_x, marker_y, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
end


