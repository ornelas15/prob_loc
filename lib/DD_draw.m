function dd_d = DD_draw(X, Y, ang)
% Function to plot the DD robot
%
% X,Y: Center point of differential drive robot
% ang: Rotation angle in degrees (optional, default 0)
% dd_d: returns the patch of the robot
% ------------------------------------------------------------------------


if nargin < 3
    ang = 0;
end

% Default size of robot
width = 16;
height = 10;

% Bottom left corner of the rectangle
robot_x = X - width/2;
robot_y = Y - height/2;

% Generate vertices of robot
vertices = [robot_x robot_y; robot_x+width robot_y; robot_x+width robot_y+height; robot_x robot_y+height];

% Center of robot
center = [X Y];
% Rotation of the vertices 
rotation = [cosd(ang) -sind(ang); sind(ang) cosd(ang)];
vertices = (vertices - center) * rotation + center;

% Plot the DD robot
dd_d = patch('Vertices', vertices, 'Faces', [1 2 3 4], 'FaceColor', '#D95319');

% Size of wheel from robot
w_width = 4;
w_height = 2;

% Bottom left corner of the wheel1
w1_x = X-w_width/2;
w1_y = Y-height/2-w_height;
% Generate vertices of wheel1
wheel1_vert = [w1_x w1_y; w1_x+w_width w1_y; w1_x+w_width w1_y+w_height; w1_x w1_y+w_height];
wheel1_vert = (wheel1_vert - center)* rotation + center;

% Bottom left corner of the wheel2
w2_x = X-w_width/2;
w2_y = Y+height/2;
% Generate vertices of wheel2
wheel2_vert = [w2_x w2_y; w2_x+w_width w2_y; w2_x+w_width w2_y+w_height; w2_x w2_y+w_height];
wheel2_vert = (wheel2_vert - center)* rotation + center;

% Plot wheels from robot
dd_w1 = patch('Vertices', wheel1_vert, 'Faces', [1 2 3 4], 'FaceColor', '#0072BD');
dd_w2 = patch('Vertices', wheel2_vert, 'Faces', [1 2 3 4], 'FaceColor', '#0072BD');

end
