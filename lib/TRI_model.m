function [TRI_w,TRI_alpha] = TRI_model(x,y,theta, L, r, Dt)
% Function to get the angular velocity and direction of the Tricycle wheel
%
% x: estimated trajectory coordinate x
% y: estimated trajectory coordinate y
% theta: estimated trajectory orientation theta
% L: separation/distance of the wheels
% r: radius of the robots wheels
% Dt: time interval of samplings
% TRI_w: angular velocity of the rear robot wheels
% TRI_alpha: direction of the robot steering wheel
% ------------------------------------------------------------------------

% Variables to save output 
TRI_w = [];
TRI_alpha = [];

% Get number of steps
num_steps = length(x);

% Calculate angular velocities of the wheels for each step:
for n = 1:num_steps-1

    % Calculate the distance between each point
    dist_p = norm([x(n+1)-x(n), y(n+1)-y(n)]);
  
    % Calculate the angle between each point
    angle_p = theta(n+1)-theta(n);  %alpha

    % Calculate linear and angular velocity measurements
    linvel_p = dist_p/Dt;
    angvel_p = angle_p/Dt;

    % Radius of curvature
    TRI_alpha_p = atan(L*angvel_p/linvel_p);
    R = angvel_p*L;
    
    % Calculate the angular velocity wT
    TRI_w_p = linvel_p-R;  

    % Store variables
    TRI_w = [TRI_w; TRI_w_p];
    TRI_alpha = [TRI_alpha; TRI_alpha_p];

end


end