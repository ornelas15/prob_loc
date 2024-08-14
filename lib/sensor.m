function [sensor_data]=sensor(lm_p, n_state, obsNoise)
% z1 - matrix of new sensor data
% xym - vector of landmark position
% xstate1 - vector of robot pose (state)
% ndnphi - vector of uncertainties in measurement (observation noise)

sensor_data(1)=sqrt((lm_p(1)-n_state(1))^2+(lm_p(2)-n_state(2))^2)+obsNoise(1);
sensor_data(2)=atan2(lm_p(2)-n_state(2),lm_p(1)-n_state(1))-n_state(3)+obsNoise(2);

end