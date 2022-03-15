function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
kD1 = 2;
kD2 = 2;
kD3 = 16;
kDphi = 2.8;
kDtheta = 2.8;
kDpsi = 2.2;

kP1 = 6;
kP2 = 6;
kP3 = 120;
kPphi = 260;
kPtheta = 115;
kPpsi = 115;



r1_ddot_des = des_state.acc(1,1) + kD1*(des_state.vel(1,1) - state.vel(1,1)) + kP1*(des_state.pos(1,1) - state.pos(1,1));
r2_ddot_des = des_state.acc(2,1) + kD2*(des_state.vel(2,1) - state.vel(2,1)) + kP2*(des_state.pos(2,1) - state.pos(2,1));
r3_ddot_des = des_state.acc(3,1) + kD3*(des_state.vel(3,1) - state.vel(3,1)) + kP3*(des_state.pos(3,1) - state.pos(3,1));

phi_des = (1/params.gravity)*(r1_ddot_des*sin(des_state.yaw) - r2_ddot_des*cos(des_state.yaw));
theta_des = (1/params.gravity)*(r1_ddot_des*cos(des_state.yaw) + r2_ddot_des*sin(des_state.yaw));
psi_des = des_state.yaw;

% Thrust
F = params.mass*params.gravity + params.mass*r3_ddot_des;


% Moment
m = [kPphi*(phi_des - state.rot(1,1)) + kDphi*(-state.omega(1,1));kPtheta*(theta_des - state.rot(2,1)) + kDtheta*(-state.omega(2,1));kPpsi*(psi_des - state.rot(3,1)) + kDpsi*(-state.omega(3,1))];
M = params.I*m;




% =================== Your code ends here ===================

end
