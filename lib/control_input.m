function [vel_noise,dist_point] = control_input(num_steps,traj_points,delta_t,sigma_V,sigma_W) 
% Function to generate the control input for the EKF
%
% num_steps: Number of trajectory steps
% traj_points: Points of the trajectory
% delta_t: time interval assumed 
% sigma_V: uncertainty of the linear velocity of the robot
% sigma_W: uncertainty of the angular velocity of the robot
% vel_noise: returns output of both linear and angular 
% velocities with noise: [V,W]
% ------------------------------------------------------------------------


% Variable for distance points
dist_points = [];

% Variable for angle between points
dang_points = [];

% Variables for velocities
linvel = [];
angvel = [];

for num = 1:num_steps

    % Calculate the distance between each point
    dist_point = norm([traj_points(num+1,1) - traj_points(num,1), traj_points(num+1,2) - traj_points(num,2)]);
  
    % Calculate the angle between each point
    angle_point = traj_points(num+1,3) - traj_points(num,3);

    % Calculate linear and angular velocity measurements
    linvel_point = dist_point/delta_t;
    angvel_point = angle_point/delta_t;
    
    % Save distance and angle difference between points
    dist_points = [dist_points;dist_point];
    dang_points = [dang_points;angle_point];

    % Save velocities values
    linvel = [linvel;linvel_point];
    angvel = [angvel;angvel_point];

end

% Add noise to the linear and angular velocity measurements
noise_V = sigma_V * randn(num_steps,1);
noise_W = sigma_W * randn(num_steps,1);
linvel_noise = linvel + noise_V;
angvel_noise = angvel + noise_W;

% Save velocities and noise
% noise = [noise_V,noise_W];
% vel = [linvel,angvel];
vel_noise = [linvel_noise,angvel_noise];


end