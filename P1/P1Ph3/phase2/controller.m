function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

% Desired roll, pitch and yaw
psi_des = 0;

% 
kp = [10,10,11];
kd = [4,4,4];
kdAng = [5,5,3];
kpAng = [1000,1000,1000];
% Known Information to the controller
state = qd{qn};
des_state = qd{qn};
% Thurst
%F = 0;
F = params.mass * (params.grav + des_state.acc_des(3) + kd(3) * (des_state.vel_des(3) - state.vel(3)) + kp(3) * (des_state.pos_des(3) - state.pos(3)));
ycddot = des_state.acc_des(2) + kd(2) * (des_state.vel_des(2) - state.vel(2)) + kp(2) * (des_state.pos_des(2) - state.pos(2));
xcddot = des_state.acc_des(1) + kd(1) * (des_state.vel_des(1) - state.vel(1)) + kp(1) * (des_state.pos_des(1) - state.pos(1));
phi_des = (xcddot * sin(des_state.yaw_des) - ycddot * cos(des_state.yaw_des)) / (params.grav);
theta_des = (xcddot * cos(des_state.yaw_des) + ycddot * sin(des_state.yaw_des))/ (params.grav);

%Moment
M = zeros(3,1);
M(1,1) = (kpAng(1) * (phi_des - state.euler(1)) + kdAng(1) * ( - state.omega(1)));
M(2,1) = (kpAng(2) * (theta_des - state.euler(2)) + kdAng(2) * ( - state.omega(2)));
M(3,1) = (kpAng(3) * (des_state.yaw_des - state.euler(3)) + kdAng(3) * ( - state.omega(3)));  %des_state.yaw_dot
% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end

