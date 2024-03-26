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
origin = [500 500 0];
figure;
hold on;
axis([0 2000 0 2000]);
%% Robots Dimensions
robot_size = [138 178 192]; % L x W x H in mm
Tyre_diameter = 66; % mm
Tyre_Width = 20; % mm for visualization
Tyre_distance = 160; % mm distance between tyres
%% Max Speeds
v_max = 0.22; %mm/s max translational velocity
w_max = 2.84; %rad/s max angular velocity
%% Inverse Kinematics
R = Tyre_diameter / 2;
L = Tyre_distance / 2;

%% Define Path
% Define starting and ending points
start_point = [500 500 0];
end_point = [1400 1400 0];

% Define the number of points in the path
num_points = 16;

% Generate the x-coordinates using a sinusoidal function
x_values = linspace(start_point(1), end_point(1), num_points);
x_values = x_values + sin(linspace(0, pi, num_points)) * 50; % Adjust amplitude as needed

% Generate the y-coordinates using a sinusoidal function
y_values = linspace(start_point(2), end_point(2), num_points);
y_values = y_values + sin(linspace(0, pi, num_points)) * 500; % Adjust amplitude as needed

% Combine the x, y, and theta values into the path matrix
path = [x_values' y_values'];
left_tyre_pos = [500; 500+Tyre_distance/2]; 
right_tyre_pos = [500; 500-Tyre_distance/2]; 
%% Open-loop Control
dt = 1; % Time step
for i = 1:size(path, 1)
    % Extract desired position and orientation from the path
    
    desired_position = path(i, 1:2);
    desired_orientation = atan2(path(i+1, 2) - desired_position(2), path(i+1, 1) - desired_position(1));
    


    % Calculate desired velocities based on the difference between current and desired positions
    vx = (desired_position(1) - origin(1)) / dt; 
    vy = (desired_position(2) - origin(2)) / dt; 
    desired_orientation_diff = desired_orientation - pi/2; 
    w = desired_orientation_diff / dt; 

     % Calculate wheel velocities using inverse kinematics
    wheel_velocities = inverse_kinematics(vx, vy, w, R, L);
    vr = wheel_velocities(1);
    vl = wheel_velocities(2);
    w = wheel_velocities(3);
    
    % Update tire positions based on respective wheel velocities
    left_tyre_pos = left_tyre_pos + vl * [cos(desired_orientation); sin(desired_orientation)] * dt;
    right_tyre_pos = right_tyre_pos + vr * [cos(desired_orientation); sin(desired_orientation)] * dt;

    % Calculate new origin based on the average position of tires
    origin = [(left_tyre_pos(1) + right_tyre_pos(1)) / 2, (left_tyre_pos(2) + right_tyre_pos(2)) / 2, origin(3)];
    
    
    % Update plot
    cla;
    plot(path(:, 1), path(:, 2), 'r--'); % Plot path
    plot(origin(1),origin(2),"x")
    circle_radius = 2;
    front_circle_x = origin(1) + (robot_size(1)/2 + circle_radius) * cos(desired_orientation);
    front_circle_y = origin(2) + (robot_size(1)/2 + circle_radius) * sin(desired_orientation);
    plot(front_circle_x, front_circle_y, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    
    rectangle_rotation = rad2deg(desired_orientation);

% Define the vertices of the rectangle
rect_vertices = [origin(1)-robot_size(1)/2, origin(2)-robot_size(2)/2;  % Bottom-left
                 origin(1)+robot_size(1)/2, origin(2)-robot_size(2)/2;  % Bottom-right
                 origin(1)+robot_size(1)/2, origin(2)+robot_size(2)/2;  % Top-right
                 origin(1)-robot_size(1)/2, origin(2)+robot_size(2)/2]; % Top-left

% Plot robot body as a rectangle
robot_patch = patch('Vertices', rect_vertices, 'Faces', [1 2 3 4], ...
                    'FaceColor', 'none', 'EdgeColor', 'b');

% Rotate the rectangle to the desired orientation
rotate(robot_patch, [0 0 1], rad2deg(desired_orientation), origin);
    
    % Calculate tire positions relative to rectangle's center
    left_tyre_origin = [-robot_size(1)/2+14, 18+Tyre_distance/2 ];
    right_tyre_origin = [-robot_size(1)/2+14, 18-(Tyre_distance/2 )];
    % Rotate tire positions
    rotation_matrix = [cos(desired_orientation) -sin(desired_orientation); sin(desired_orientation) cos(desired_orientation)];
    rotated_left_tyre_origin = rotation_matrix * left_tyre_origin' + [origin(1); origin(2)];
    rotated_right_tyre_origin = rotation_matrix * right_tyre_origin' + [origin(1); origin(2)];
    % Plot tires
    viscircles([rotated_left_tyre_origin(1), rotated_left_tyre_origin(2)], Tyre_Width)
    viscircles([rotated_right_tyre_origin(1), rotated_right_tyre_origin(2)], Tyre_Width)
    
    % Pause for visualization
    drawnow;
    pause(dt);
end
hold off;

function result = inverse_kinematics(vx, vy, w, R, L)
    Tr = (vx + vy + L * w) / R;
    Tl = (vx + vy - L * w) / R;
 
    vr = 0.5 * R * (Tr + Tl) + 0.5 * L * R * (Tr - Tl);
    vl = 0.5 * R * (Tr + Tl) - 0.5 * L * R * (Tr - Tl);
    
    result = [vr, vl, w];
end

function result=forward_kinematics(vr,vl,R,L,theta)
%%%Not Used
    tyre_matrix = [vr ; vl];
    middle_matrix = [R*cos(theta)/2 , R*cos(theta)/2 ; R*sin(theta)/2, R*sin(theta)/2 ; R/(2*L), -R/(2*L) ];
    outcome_matrix= middle_matrix*tyre_matrix;
    xn=outcome_matrix(1);
    yn=outcome_matrix(2);
    wn=outcome_matrix(3);
    result=[xn yn wn];
end
