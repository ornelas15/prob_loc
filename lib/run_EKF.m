function [state_predicted,CovM_predicted,obs_beacons_step] = run_EKF(num_steps,control_vel,obs_data,landmark_points,delta_t,Q,R_i)

% Create estimated state and covariance matrix variables
state_predicted = [0, zeros(1,3)];   
CovM_predicted = 0.01*eye(3);

% Get number of beacons used in each observation
obs_beacons_step = [];

% Run filter on trajectory points
for step = 1:num_steps  

    % EKF estimate at time t
    state_t = state_predicted(end,2:4)';
    CovM_t = CovM_predicted(end-2:end,:);
    
    % Control input at time t (step - V,W)
    control_t = control_vel(step,:);
    
    % Observation data at time t+1
    obs_data_t1 = obs_data{step,1};
   
    % Find num_beacons used for observation
    obs_beacons = size(obs_data_t1,2)/3;  % for every beacon obs_data returns 3 values: (id,x,y)
    
    % concatenate R_i N times
    R = zeros(2*obs_beacons);
    for i = 1:obs_beacons
        R((i-1)*2+1:i*2, (i-1)*2+1:i*2) = R_i;
    end
    
    % Get number of beacons used in each observation
    obs_beacons_step = [obs_beacons_step,obs_beacons];

    % Using ekf function    
    [state_predicted_t,CovM_predicted_t] = EKF(state_t,CovM_t,control_t,obs_data_t1,landmark_points,delta_t,Q,R);
    
    % Update
    state_predicted = [state_predicted; step, state_predicted_t];
    CovM_predicted = [CovM_predicted; CovM_predicted_t];

end

end