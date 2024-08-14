function obs_data = observation(num_beacons,traj_points,obsNoise_dist,obsNoise_ang) 
% Function to generate the observation data for the EKF
%
% num_beacos: Number of beacons used for observation
% traj_points: Points of the trajectory
% obsNoise_dist: uncertainty of the distance of the observation
% obsNoise_ang: uncertainty of the orientation of the observation
% ------------------------------------------------------------------------


% Define num of points
num_points = size(traj_points,1);

% Create measures variable
measures = [];
range = [];
bearing = [];
obs_data = cell(num_points, 1);

% Call BeaconDetection for each point, to read measures
for point = 1:num_points

    % Get measures for each trajectory point
    meas = BeaconDetection(num_beacons,traj_points(point,:),[obsNoise_dist,obsNoise_ang]);  

    % Add number of beacons and beacon id to measures structure
    for beacons = 1:size(meas,2)
        meas(beacons).step = point;
        meas(beacons).bid = beacons;
    end
    
    % Change order of measures fields
    meas = orderfields(meas, {'step', 'bid', 'X', 'Y', 'd', 'a', 'dn', 'an'});
    
    % For each point get max possible beacons for obs
    obs_meas = meas;
    
    % Remove NaN measures from beacons
    check_ids = ~isnan([obs_meas.d]);
   
    % Get corresponding struct elements
    obs_meas = obs_meas(check_ids);

    % Initialize temporary variables to store bid, range, and bearing
    bid_p = [];
    range_p = [];
    bearing_p = [];
    obs_data_p = [];
    
    % Loop over all observations and extract bid, range, and bearing
    for id = 1:size(obs_meas,2)
        bid_p(id) = obs_meas(id).bid;
        range_p(id) = obs_meas(id).d;
        bearing_p(id) = obs_meas(id).a;
        obs_data_p = [obs_data_p,bid_p(id),range_p(id),bearing_p(id)];
    end

    % Save all observation data for each point
    obs_data{point} = obs_data_p;
   
end

end