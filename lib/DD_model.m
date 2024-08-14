function [DD_wr,DD_wl] = DD_model(x,y,theta, L,Dt)
% Function to get the angular velocities of the DD left and right wheels
%
% x: estimated trajectory coordinate x
% y: estimated trajectory coordinate y
% theta: estimated trajectory orientation theta
% L: separation/distance of the wheels
% r: radius of the robots wheels
% Dt: time interval of samplings
% DD_wl: angular velocity of the robot left wheel
% DD_wr: angular velocity of the robot right wheel
% ------------------------------------------------------------------------

% Variables to save output 
DD_wl = [];
DD_wr = [];

% Get number of steps
num_steps = length(x);

% Calculate angular velocities of the wheels for each step:
for n = 1:num_steps-1

    % Calculate the distance between each point
    dist_p = norm([x(n+1)-x(n), y(n+1)-y(n)]);
  
    % Calculate the angle between each point
    angle_p = theta(n+1)-theta(n);

    if angle_p == 0
        % linear velocity of the wheels
        linvel_p = dist_p/Dt;
        DD_wr_p = linvel_p;
        DD_wl_p = linvel_p;
    
    else
        % radius of curvature
        R = dist_p/angle_p;
        
        % Calculate the angular velocities of the wheels
        DD_wr_p = (2*angle_p*R+L*angle_p)/(2*R);
        DD_wl_p = (2*angle_p*R-L*angle_p)/(2*R);
    end
  
    % Store variables
    DD_wl = [DD_wl; DD_wl_p];
    DD_wr = [DD_wr; DD_wr_p];

end


end
