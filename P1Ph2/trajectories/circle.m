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

total_time = 12;
t_interval = total_time / 3;

%vmax = z / (2 * t_interval);
%amax = vmax / (t_interval);
num_circles = 1;

wmax = (3 * pi) * num_circles /  (total_time);
alphamax = wmax / t_interval;

tmax = total_time;
%omega = 2 * pi * num_circles / (tmax);
A = 5;
dt = 0.0001;
    function pos = getPose(angle)
        pos = [A * cos(angle); A * sin(angle); z * angle / (2 * pi * num_circles)];
    end
    
    function angle = getAngleFromTrapzProfile(t)
        if t <= tmax / 3
            angle = 0.5 * ((3 * wmax) / tmax) * t ^2;
        elseif t <= 2 * tmax / 3
            deltat = t - tmax/3;
            angle = (wmax^2) / (2 * alphamax) + wmax * deltat;
        elseif t <= tmax
            deltat = t - 2 * tmax / 3;
            angle = 3 * (wmax^2) / (2 * alphamax) + wmax*deltat - 0.5*alphamax*deltat^2;
        else
            angle = 2*(wmax^2)/alphamax;
        end
    end
    function vel = getVelocity(t)
        angle = getAngleFromTrapzProfile(t);
        angle2 = getAngleFromTrapzProfile(t + dt);
        pos_1 = getPose(angle);
        pos_2 = getPose(angle2);
        vel = (pos_2 - pos_1) / dt;
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
    angle = getAngleFromTrapzProfile(t);
    pos = getPose(angle);
    vel = getVelocity(t);
    acc = (getVelocity(t + dt) - getVelocity(t)) / dt;
else
    pos = [A * cos(2*pi*num_circles); A * sin(2*pi*num_circles); z ];
    vel = zeros(3,1);
    acc = zeros(3,1);
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
