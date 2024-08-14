function rm1_109802(varargin)
% Nome: David Ornelas;
% N_mec: 109802
%
% Run on command line: 
% - rm1_109802()                                    use default values
% - rm1_109802([],'x',[],[],[],[],[],[],[])         change several values
% - rm1_109802('delta_t', 3)                        change specific value
%
% Input order: (num_beacons, delta_t, radius_wh, dist_wh, ...
%                   sigma_V, sigma_W, obsNoise_dist, obsNoise_ang, i_mode)
%
%
% Run interactive mode: rm1_109802('i_mode', 1) 
%
% num_beacons - nº of beacons, def = 4
% delta_t - time sampling of sensors, default = 1 s
% radius_wh - wheel radius, def = 0.15 m
% dist_wh - wheel distancing, def = 1 m
% sigma_V - uncertainty of linear velocity, def = 0.1 m/s
% sigma_W - uncertainty of angular velocity, def = 0.1 m/s
% obsNoise_dist - obsNoise distance, def = 0.25 m
% obsNoise_ang - obsNoise angle, def = 0.1 rad
% i_mode - interactive mode option, def = 0.    (ON = 1).

addpath lib/
clc; close all;

%% Default values - 
% To change any default value into 'x' write the following comand line:
% rm1_109802([], x, [], [], [], [], [], [],[]).

% Define default values
def_num_beacons = 4;
def_delta_t = 1;
def_radius_wh = 0.15;
def_dist_wh = 1;
def_sigma_V = 0.1;
def_sigma_W = 0.1;
def_linear_vel = 5;
def_obsNoise_dist = 0.25;
def_obsNoise_ang = 0.1;
% Interactive mode option
def_i_mode = 0;

% Create input parser
p = inputParser;

% Define optional inputs with default values
p.addOptional('num_beacons', def_num_beacons, @(x) validateattributes(x, {'numeric'}, {'integer'}));
p.addOptional('delta_t', def_delta_t, @(x) validateattributes(x, {'numeric'}, {'scalar'}));
p.addOptional('radius_wh', def_radius_wh, @(x) validateattributes(x, {'numeric'}, {'scalar'}));
p.addOptional('dist_wh', def_dist_wh, @(x) validateattributes(x, {'numeric'}, {'scalar'}));
p.addOptional('sigma_V', def_sigma_V, @(x) validateattributes(x, {'numeric'}, {'scalar'}));
p.addOptional('sigma_W', def_sigma_W, @(x) validateattributes(x, {'numeric'}, {'scalar'}));
p.addOptional('linear_vel', def_linear_vel, @(x) validateattributes(x, {'numeric'}, {'scalar'}));
p.addOptional('obsNoise_dist', def_obsNoise_dist, @(x) validateattributes(x, {'numeric'}, {'scalar'}));
p.addOptional('obsNoise_ang', def_obsNoise_ang, @(x) validateattributes(x, {'numeric'}, {'scalar'}));
p.addOptional('i_mode', def_i_mode, @(x) validateattributes(x, {'numeric'}, {'integer'}));

% Parse inputs
parse(p, varargin{:});     

% Load default values of inputs 
num_beacons = p.Results.num_beacons;
delta_t = p.Results.delta_t;
radius_wh = p.Results.radius_wh;
dist_wh = p.Results.dist_wh;
sigma_V = p.Results.sigma_V;
sigma_W = p.Results.sigma_W;
linear_vel = p.Results.linear_vel;
obsNoise_dist = p.Results.obsNoise_dist;
obsNoise_ang = p.Results.obsNoise_ang;
i_mode = p.Results.i_mode;

% Avoid nº of beacons below 4
if num_beacons < 4
    error('The minimum number of beacons required are 4. Please run the program with at least 4.');
end

%% Get landmarks for observation

% Display message:
if i_mode == 1
    disp('Starting program...')
    disp('---------------------------')
    pause(2)
    disp('Reading sensors...')
    disp('---------------------------')
    pause(2)
end


% Spawn landmarks
measures = BeaconDetection(num_beacons);

% Create landmark variables
landmark_x = [];
landmark_y = [];
landmark_points = [];

% Get coordinates from landmarks
for num=1:num_beacons
    landmark_x(num,:) = measures(num).X;
    landmark_y(num,:) = measures(num).Y;
    landmark_points(num,:) = [num,landmark_x(num,:),landmark_y(num,:)]; 
end

%% Calculate trajectory points and plot trajectory

if i_mode == 1
    disp('Information retrieved. Spawning robot on initial position (0,0,0).')
    disp('---------------------------')
    pause(2)
    disp('Calculating trajectory points...')
    disp('---------------------------')
    pause(5)
end

% Get trajectory points: 
traj_points = get_traj(landmark_points, linear_vel, delta_t);


%% Calculate the linear and angular velocity at each point of the trajectory. 

% Define num of steps
num_steps = size(traj_points,1)-1;

if i_mode == 1
    disp([num2str(num_steps),' points calculated for the trajectory with ', num2str(num_beacons), ' landmarks.'])
    disp('---------------------------')
    pause(2)
    disp('Evaluating velocities for each point...')
    disp('---------------------------')
    pause(5)
end

% Generate control input data
[vel_noise,dist_point] = control_input(num_steps,traj_points,delta_t,sigma_V,sigma_W);


%% Generate observation data

if i_mode == 1
    disp('Velocities retrieved. Proceeding to the next step.')
    disp('---------------------------')
    pause(2)
    disp(['Generating observation data with ', num2str(num_beacons), ' beacons.'])
    disp('---------------------------')
    pause(5)
end

% Remove initial point to estimate future points
traj_points_obs = traj_points(2:end,:);

% Get observation data from beacons on current position
obs_data = observation(num_beacons,traj_points_obs,obsNoise_dist,obsNoise_ang);

%%  Run Extended Kalman Filter to estimate position

if i_mode == 1
    disp('Load pre process data for Extended Kalman Filter')
    disp('---------------------------')
    pause(4)
    disp(['Run Extended Kalman Filter to estimate robot position for ', num2str(num_steps), ' steps.'])
    disp('---------------------------')
    pause(8)
end

% EKF PRE PROCESS:
add_steps = (0:size(traj_points,1)-1)';

% Add steps to trajectory points:
traj_points = [add_steps,traj_points];

% Input (velocity) noise covariance matrix
Q = [sigma_V^2  0
     0   sigma_W^2]; 

% Landmark observation noise covariance matrix
R_i = [obsNoise_dist^2  0
        0   obsNoise_ang^2]; 

% Get number of beacons used for observation on each step
obs_beacons = [];

% Run Extended Kalman Filter
[state_predicted,CovM_predicted,obs_beacons] = run_EKF(num_steps,vel_noise,obs_data,landmark_points,delta_t,Q,R_i);

if i_mode == 1
    disp('Filter runned successfully')
    disp('---------------------------')
    pause(2)
    disp('Calculate Angular velocities for the robot kinematic models')
    disp('---------------------------')
    pause(5)
end

%% Calculate angular velocities based on the kinematic models with EKF points

% Calculate angular velocities of DD robot
[DD_wr,DD_wl] = DD_model(state_predicted(:,2),state_predicted(:,3),state_predicted(:,4),dist_wh,delta_t);

% Calculate angular velocity and direction of wheel of TRI robot
[TRI_w,TRI_alpha] = TRI_model(state_predicted(:,2),state_predicted(:,3),state_predicted(:,4),dist_wh,radius_wh,delta_t);

if i_mode == 1
    disp('Evaluation complete.')
    disp('---------------------------')
    pause(2)
    disp('Save and plot data for analysis!')
    disp('---------------------------')
    pause(2)
end

%% Save data 

% Create estimated state matrix:
state_M = state_predicted(:,2:4);

% Create DD matrix:
DD_info = [DD_wr,DD_wl];

% Create TRI matrix:
TRI_info = [TRI_w, TRI_alpha];

% Save estimated state on txt
writematrix(state_M, 'loc_109802.txt', 'Delimiter', ',');

% Save angular velocities of the wheels of DD robot  
writematrix(DD_info, 'DD_109802.txt', 'Delimiter', ',');

% Save angular velocities and direction of the tricycle wheel of TRI robot
writematrix(TRI_info, 'TRI_109802.txt', 'Delimiter', ',');

if i_mode == 1
    disp('Save complete.')
    disp('---------------------------')
    pause(2)
    disp('Program run complete.')
end

%% Plot data on interactive mode

if i_mode == 1
    % -------------------------------------------------------------------------
    
    % Plot trajectory
    figure(1);
    h1 = plot(traj_points(:,2), traj_points(:,3), 'bo', 'MarkerSize', 6,'MarkerFaceColor','k');
    hold on;
    % Plot landmarks
    plot(landmark_x, landmark_y, 'o', 'MarkerSize', 24,'MarkerFaceColor', 'y'); 
    h3 = plot(landmark_x, landmark_y, 'bo', 'MarkerSize', 18,'MarkerFaceColor', 'r'); hold on;
    grid on; hold on;
    title('Landmarks and trajectory points');
    xlabel('X (m)'); ylabel('Y (m)'); 
    legend([h1, h3], {'Trajectory Points', 'Landmarks'}, 'Location', 'northwest');
    
    % -------------------------------------------------------------------------
    
    % Plot control input
    figure(2)
    title('Velocities for each point'); hold on;
    title('Linear and angular velocity w/ noise'); hold on;
    h4 = plot(vel_noise(:,1),'LineWidth', 3,'Color','#77AC30'); hold on;
    h5 = plot(vel_noise(:,2),'LineWidth', 3,'Color','b'); hold on;
    grid on; hold on;
    xlabel('Time (s)'); ylabel('Velocities'); 
    legend([h4, h5], {'Linear velocity (m/s)', 'Angular velocity (rad/s)'});
    
    % -------------------------------------------------------------------------

    % Plot estimated points and trajectory points
    figure(3)
    hold on;
    h7 = plot(state_predicted(:,2),state_predicted(:,3),'.','MarkerSize',20, 'Color','r');
    grid on; hold on;
    h8 = plot(traj_points(:,2),traj_points(:,3),'-','linewidth',1, 'Color','k');
    hold on;
    plot(landmark_x, landmark_y, 'o', 'MarkerSize', 24,'MarkerFaceColor', 'y'); 
    h9 = plot(landmark_x, landmark_y, 'bo', 'MarkerSize', 18,'MarkerFaceColor', 'r'); hold on;
    title('Estimated points from EKF');
    xlabel('X (m)'); ylabel('Y (m)'); 
    legend([h7, h8, h9], {'EKF measure', 'Trajectory path','Landmarks'}, 'Location', 'northwest');
    
    % -------------------------------------------------------------------------
        
    % Plot nº of beacons used on each obs
    figure(4)    
    h14 = bar(1:num_steps, obs_beacons);
    hold on; grid on;
    title('Observation data beacons used');
    xlabel('Steps(s)'); ylabel('Nº of Beacons'); 
    legend(h14, {'Nº of beacons'}, 'Location', 'northwest');
    axis([0 num_steps 0 15])

    % -------------------------------------------------------------------------

    % Spawn robots
    %DD_d = DD_draw(0,0,0);
    %TRI_d = TRI_draw(10,10,0);
    %hold on;
    
    % Plot angular velocities of wheels for DD and TRI robot
    figure(5)
    h10 = plot(DD_wr,'-','linewidth',2, 'Color','b');
    hold on;
    h11 = plot(DD_wl,'-','linewidth',2, 'Color','r');
    hold on; grid on;
    title('Angular velocity of left and right wheel of DD robot');
    xlabel('Time(s)'); ylabel('W (rad/s)'); 
    legend([h10, h11], {'Right wheel w', 'Left wheel w'}, 'Location', 'northwest');
    
    figure(6)
    subplot(2,1,1)
    h12 = plot(TRI_w,'-','linewidth',2, 'Color','b');
    hold on; grid on;
    title('Angular velocity of rear wheels of TRI robot');
    xlabel('Time(s)'); ylabel('W (rad/s)'); 
    legend(h12, {'W (rad/s)'}, 'Location', 'northwest');
    subplot(2,1,2)
    h13 = plot(TRI_alpha,'-','linewidth',2, 'Color','r');
    hold on; grid on;
    title('Direction of the steering wheel of TRI robot');
    xlabel('Time(s)'); ylabel('W (rad/s)'); 
    legend(h13, {'alpha (rads)'}, 'Location', 'northwest');
end

end