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

%u1 = 0;
%u2 = 0;

% FILL IN YOUR CODE HERE
%E Tola
kp_phi = 500;
kd_phi = 50;
kp_y = 150;
kd_y = 60;
kp_z = 200;
kd_z = 60;

e_y = [des_state.pos(1) - state.pos(1); des_state.vel(1) - state.vel(1)];
e_z = [des_state.pos(2) - state.pos(2); des_state.vel(2) - state.vel(2)];

u1  = params.mass*(params.gravity + des_state.acc(2)+ kd_z*e_z(2) + kp_z*e_z(1));
phi_c = -(1/params.gravity)*(des_state.acc(1)+kd_y*e_y(2)+kp_y*e_y(1));
e_phic = [phi_c - state.rot;-state.omega];
u2 = kp_phi*e_phic(1)+kd_phi*e_phic(2);
end

