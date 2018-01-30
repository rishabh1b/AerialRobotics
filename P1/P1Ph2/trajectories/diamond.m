function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables

pos = [0; 0; 0];
vel = [0; 0; 0];
acc = [0; 0; 0];
yaw = 0;
yawdot = 0;

total_dist_x = 1;
p = 1;

% Splitting total time into four intervals
t_total = 11 * p;

t_interval = t_total / 4;

t_inc_dec = t_interval / 2;


vmax_x = total_dist_x /(4 * t_inc_dec);
vmax_yz = p / t_inc_dec;

a_x = vmax_x / t_inc_dec;
a_yz = vmax_yz / t_inc_dec;

if t <= t_inc_dec
    pos(1) = 0.5 * a_x * (t ^ 2);
    vel(1) = a_x * t;
    acc(1) = a_x;
    pos(2) = 0.5 * a_yz * (t ^ 2);
    vel(2) = a_yz * t;
    acc(2) = a_yz;
    pos(3) = 0.5 * a_yz * (t ^ 2);
    vel(3) = a_yz * t;
    acc(3) = a_yz;
elseif t <= 2 * t_inc_dec
    deltat = t - t_inc_dec;
    pos(1) = (vmax_x^2) / (2 * a_x) + vmax_x*deltat - 0.5*a_x*deltat^2;
    vel(1) = vmax_x - a_x * deltat;
    acc(1) = -a_x;
    pos(2) = (vmax_yz^2) / (2 * a_yz) + vmax_yz*deltat - 0.5*a_yz*deltat^2;
    vel(2) = vmax_yz - a_yz * deltat;
    acc(2) = -a_yz;
    pos(3) = (vmax_yz^2) / (2 * a_yz) + vmax_yz*deltat - 0.5*a_yz*deltat^2;
    vel(3) = vmax_yz - a_yz * deltat;
    acc(3) = -a_yz;
elseif t <= 3 * t_inc_dec 
    deltat = t - 2 * t_inc_dec;
    pos(1) = (vmax_x^2) / (a_x) + 0.5 * a_x * (deltat ^ 2);
    vel(1) = a_x * deltat;
    acc(1) = a_x;
    pos(2) = (vmax_yz^2) / (a_yz) - 0.5*a_yz*deltat^2;
    vel(2) = -a_yz * deltat;
    acc(2) = -a_yz;
    pos(3) = (vmax_yz^2) / (a_yz) + 0.5 * a_yz * (deltat ^ 2);
    vel(3) = a_yz * deltat;
    acc(3) = a_yz;
elseif t <= 4 * t_inc_dec
    deltat = t - 3 * t_inc_dec;
    pos(1) = (3 * vmax_x^2) / (2 * a_x) + vmax_x*deltat - 0.5*a_x*deltat^2;
    vel(1) = vmax_x - a_x * deltat;
    acc(1) = -a_x;
    pos(2) = (vmax_yz^2) / (2*a_yz)  - vmax_yz*deltat + 0.5*a_yz*deltat^2;
    vel(2) = -vmax_yz + a_yz * deltat;
    acc(2) = a_yz;
    pos(3) = (3 * vmax_yz^2) / (2 * a_yz) + vmax_yz*deltat - 0.5*a_yz*deltat^2;
    vel(3) = vmax_yz - a_yz * deltat;
    acc(3) = -a_yz;
elseif t <= 5 * t_inc_dec
    deltat = t - 4 * t_inc_dec;
    pos(1) = (2 * vmax_x^2) / (a_x) + 0.5 * a_x * (deltat ^ 2);
    vel(1) = a_x * deltat;
    acc(1) = a_x;
    pos(2) = -0.5*a_yz*deltat^2;
    vel(2) = -a_yz * deltat;
    acc(2) = -a_yz;
    pos(3) = (2 * vmax_yz^2) / (a_yz) - 0.5*a_yz*deltat^2;
    vel(3) = -a_yz * deltat;
    acc(3) = -a_yz;
elseif t <= 6 * t_inc_dec
    deltat = t - 5 * t_inc_dec;
    pos(1) = (5 * vmax_x^2) / (2 * a_x) + vmax_x*deltat - 0.5*a_x*deltat^2;
    vel(1) = vmax_x  - a_x * deltat;
    acc(1) = -a_x;
    pos(2) = -(vmax_yz^2) / (2 * a_yz) -vmax_yz*deltat + 0.5*a_yz*deltat^2;
    vel(2) = -vmax_yz + a_yz * deltat;
    acc(2) = a_yz;
    pos(3) = (3 * vmax_yz^2) / (2 * a_yz) -vmax_yz*deltat + 0.5*a_yz*deltat^2;
    vel(3) = -vmax_yz +a_yz * deltat;
    acc(3) = a_yz;
elseif t <= 7 * t_inc_dec
    deltat = t - 6 * t_inc_dec;
    pos(1) = (3 * vmax_x^2) / (a_x) + 0.5 * a_x * (deltat ^ 2);
    vel(1) = a_x * deltat;
    acc(1) = a_x;
    pos(2) = -(vmax_yz^2) / (a_yz) + 0.5*a_yz*deltat^2;
    vel(2) = a_yz * deltat;
    acc(2) = a_yz;
    pos(3) = (vmax_yz^2) / (a_yz)- 0.5*a_yz*deltat^2;
    vel(3) = -a_yz * deltat;
    acc(3) = -a_yz;
elseif t <= 8 * t_inc_dec
    deltat = t - 7 * t_inc_dec;
    pos(1) = (7 * vmax_x^2) / (2 * a_x) + vmax_x*deltat - 0.5*a_x*deltat^2;
    vel(1) = vmax_x  - a_x * deltat;
    acc(1) = -a_x;
    pos(2) = -(vmax_yz^2) / (2 * a_yz) + vmax_yz*deltat - 0.5*a_yz*deltat^2;
    vel(2) = vmax_yz - a_yz * deltat;
    acc(2) = -a_yz;
    pos(3) = (vmax_yz^2) / (2 * a_yz) -vmax_yz*deltat + 0.5*a_yz*deltat^2;
    vel(3) = -vmax_yz +a_yz * deltat;
    acc(3) = a_yz;
else
    pos(1) = 1;
    vel(1) = 0;
    acc(1) = 0;
    pos(2) = 0;
    vel(2) = 0;
    acc(2) = 0;
    pos(3) = 0;
    vel(3) = 0;
    acc(3) = 0;
end

    
% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
