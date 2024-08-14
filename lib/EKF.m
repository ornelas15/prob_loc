function [nstate_predicted,CovM_predicted] = EKF(n_state,CovM_t,control_vel,obs_data_t,landmark_points,delta_t,Q,R)

% Copyright (C) 2016 Shoudong Huang 
% University of Technology, Sydney, Australia
% 
% Author:  Shoudong Huang     -- Shoudong.Huang@uts.edu.au
%
% Adapted by David Ornelas (dornelas@ua.pt, May 2023),University of Aveiro.
% =========================================================================

%% Prediction step

% motion model noise
pzero =[0 0];       

% Motion model estimate
[nstate_t] = motion(n_state,control_vel,pzero,delta_t);


% Jacobian matrix for F
d_x = -delta_t*control_vel(1)*sin(n_state(3)); % Partial derivative of new x
d_y = -delta_t*control_vel(1)*cos(n_state(3)); % Partial derivative of new y

% Jacobian matrix with state variables
Jfx=[1 0 d_x
     0 1 d_y
     0 0 1     
     ];

% Partial derivatives for linear and angular noise
d_v =  delta_t*control_vel(1)*cos(n_state(3));
d_w =  delta_t*control_vel(1)*sin(n_state(3));

% Jacobian matrix with noise variables
Jfw=[d_v  0 
     d_w  0
     0    delta_t
     ];

% Predicted covariance matrix (propagation of uncertainty in the previous state estimate)
CovM_t_predicted= Jfx*CovM_t*Jfx'+Jfw*Q*Jfw';                           


%% Update step

% Find num_beacons used for observation
N = size(obs_data_t,2)/3;  % for every beacon obs_data returns 3 values: (id,x,y)

% Get N observations
obs_all = [];
for i = 1:N
    obs_all = [obs_all; obs_data_t((i-1)*3+2:(i-1)*3+3)'];
end

% get N landmark positions
landmarks = [];
for i = 1:N
    landmarks = [landmarks; landmark_points(obs_data_t((i-1)*3+1),2:3)];
end 


%set noise equal to 0
nzero = [0 0];  

% predicted observation
obs_pred = [];
for i = 1:N
    obs_pred_i = sensor(landmarks(i,:),nstate_t,nzero);
    obs_pred = [obs_pred; obs_pred_i'];
end

% Innovation
innov = obs_all-obs_pred;
% wrap the angles to [-pi, pi]
for i = 1:N
    innov((i-1)*2+2) = wrap(innov((i-1)*2+2));
end


% Jacobian matrix
Jh = [];
for i = 1:N
    Jh_i = jacobian(landmarks(i,:),nstate_t(1),nstate_t(2));
    Jh = [Jh; Jh_i];
end

% Kalman gain
S = Jh*CovM_t_predicted*Jh'+R;
K = CovM_t_predicted*Jh'*inv(S);

% Update state estimate
nstate_t_updated = nstate_t'+K*innov;

% Update covariance matrix
CovM_t_updated = CovM_t_predicted - K*Jh*CovM_t_predicted;

% Result for next prediction step
nstate_predicted = nstate_t_updated';
CovM_predicted = CovM_t_updated;

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

