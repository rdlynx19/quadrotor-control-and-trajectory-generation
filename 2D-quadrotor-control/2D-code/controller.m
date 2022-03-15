function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

edot_z = des_state.vel(2,1) - state.vel(2,1);
e_z = des_state.pos(2,1) - state.pos(2,1); 

edot_y = des_state.vel(1,1) - state.vel(1,1);
e_y = des_state.pos(1,1) - state.pos(1,1);

u1 = (params.mass)*(9.81 + des_state.acc(2,1) +10 *(edot_z) + 115*(e_z));
phi = (-1/(9.81))*(des_state.acc(1,1) + 30*(edot_y) + 1.8*(e_y));
u2 = (params.Ixx)*(0 +69* ( 0 - state.omega) + 260*(phi - state.rot));

% FILL IN YOUR CODE HERE

end

