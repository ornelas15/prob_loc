function [traj_points,n_points] = get_traj(landmark_points, linear_vel, delta_t)
% Function to get the trajectory points based on the landmarks
%
% landmark_points: (nºbeacon,x,y)
% linear_vel: velocity assumed to perform calculations
% delta_t: time interval assumed to perform calculations
% traj_points: returns calculated trajectory points
% ------------------------------------------------------------------------

% Get x and y from landmarks:
landmark_x_p = landmark_points(:,2);
landmark_y_p = landmark_points(:,3);

% Add initial robot point for calculations
landmark_p0 = [0 0;  landmark_x_p landmark_y_p];

% Get dif of each landmark distance in x and y
dif_points = diff(landmark_p0,1);

% Calculate distance of each landmark
dist_coords = sqrt(sum(power(dif_points,2), 2));

% Divide the distance in equidistant points and round
n_points = ceil(dist_coords/(linear_vel*delta_t));

% Initialize matrix to save middle x points on each trajectory action
traj_x_points = [];

% Loop over each action of the trajectory
for i = 1:length(n_points)

    % Get previous and successor beacons
    prev_beacon_x = landmark_p0(i,1);
    suc_beacon_x = landmark_p0(i+1,1);
    
    % Split in equidistant points and save middle points
    points_x = [linspace(prev_beacon_x, suc_beacon_x, n_points(i)+1)]';  % +1: preserve end points

    % Remove goal point to avoid duplicates
    if i ~= length(n_points)    
        points_x = points_x(1:end-1);    
    end

    % Save trajectory points
    traj_x_points = [traj_x_points;points_x];
    
end

% Get correspondent values of y using Hermite cubic polynomial interpolation
traj_y_points = pchip(landmark_p0(:,1),landmark_p0(:,2),traj_x_points);


for i=1:length(traj_x_points)-1

    % Difference between x
    n_xpoint = traj_x_points(i);
    next_xpoint = traj_x_points(i+1);
    %Difference between y
    n_ypoint = traj_y_points(i);
    next_ypoint = traj_y_points(i+1);
    % Update theta to each next midpoint with the atan function
    theta_points(i) = atan2(next_ypoint-n_ypoint, next_xpoint-n_xpoint);
end


% Make sure its within [-pi,pi] range
for n=1:length(traj_x_points)-1
    theta_points(i) = wrap(theta_points(i));
end

% Add initial orientation to theta points
theta_points = [0 theta_points]';

% Save all trajectory points in format (x,y,theta)
traj_points = [traj_x_points,traj_y_points,theta_points];

end


function nu = wrap(alpha)

clear nu;
nu = alpha;

	while (nu > pi)
		nu = nu - 2 * pi;
    end

	while (nu < -pi)
		nu = nu + 2 * pi;
    end
end