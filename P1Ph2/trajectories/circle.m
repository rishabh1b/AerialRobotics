function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables

pos = [0; 0; 0];
vel = [0; 0; 0];
acc = [0; 0; 0];
yaw = 0;
yawdot = 0;

z = 2.5;

total_time = 14;
t_interval = total_time / 3;

vmax = z / (2 * t_interval);
amax = vmax / (t_interval);

tmax = total_time;
num_circles = 1;
omega = 2 * pi * num_circles / (tmax);
A = 5;

if t <= tmax / 3
    pos(3) = 0.5 * amax * (t ^ 2);
    vel(3) = amax * t;
    acc(3) = amax;
elseif t <= 2 * tmax / 3
    deltat = t - tmax/3;
    pos(3) = (vmax^2) / (2 * amax) + vmax * deltat;
    vel(3) = vmax;
    acc(3) = 0;
elseif t <= tmax
    deltat = t - 2 * tmax / 3;
    pos(3) = 3 * (vmax^2) / (2 * amax) + vmax*deltat - 0.5*amax*deltat^2;
    vel(3) = vmax - amax * deltat;
    acc(3) = -amax;
else
    vel(3) = 0;
    acc(3) = 0;
    pos(3) = 2*(vmax^2)/amax;
end

buffer = 1;
t_init = buffer;
t_last = tmax - buffer;

C_start = [-3.4805, 4.5023, 1, 0];
C_end =  1.0e+03 *[
    0.0035
   -0.1417
    1.9195
   -8.6541];

if t < tmax
    pos(1) = A * cos(omega * t);
    vel(1) = -A * omega * sin(omega * t);
    acc(1) = -A * omega^2 * cos(omega * t);
    
else
    vel(1) = 0;
    acc(1) = 0;   
    pos(1) = A * cos(omega * tmax);
    vel(2) = 0;
    acc(2) = 0;
    pos(2) = A * sin(omega * tmax);
end

% if t <= t_init
%    pos(2) = A * sin(omega * t);
%    vel(2) = polyval(C_start', t);
%    acc(2) = polyval(polyder(C_start'), t); 
if t < t_last
    pos(2) = A * sin(omega * t);
    vel(2) = A * omega * cos(omega * t);
    acc(2) = -A * omega^2 * sin(omega * t);
elseif t >= t_last && t < tmax
   pos(2) = A * sin(omega * t);
   vel(2) = polyval(C_end, t);
   acc(2) = polyval(polyder(C_end), t);
end


% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
