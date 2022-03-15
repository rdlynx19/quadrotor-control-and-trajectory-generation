function [ u ] = controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

e = s_des(1,1) - s(1,1);
e_dot = s_des(2,1) - s(2,1);


u = params.mass*( 65*e + 10.5*e_dot + params.gravity);







% FILL IN YOUR CODE HERE


end

