function [next_state] = motion(n_state,vel,noise,dtime)
% xstatet1 - the next robot state (pose) (at t+1)
% xstate - the current (x,y,th) pose (at instant t)
% Vin - the linear and angular velocities (V,w)
% Dn - the uncertainty (errors) in velocities (dV, dw)
% t - the sampling time

next_state(1)=n_state(1)+(vel(1)+noise(1))*dtime*cos(n_state(3));

next_state(2)=n_state(2)+(vel(1)+noise(1))*dtime*sin(n_state(3));

next_state(3)=n_state(3)+(vel(2)+noise(2))*dtime;

end