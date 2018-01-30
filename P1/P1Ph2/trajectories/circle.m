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

num_circles = 1;

wmax = (3 * pi) * num_circles /  (total_time);
alphamax = wmax / t_interval;

tmax = total_time;
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

% This approach resorts to basic differentiation to get the velocity and 
% acceleration on the circle. The angular velocity and linear velocity 
% across z both are given a trapezoidal velocity profile. 
% The trapezoidal velocity profile for angular velocity ensures that we
% start at rest and end at rest preventing overshoot at the end. 
% This was not possible to achieve with direct sine and cosine function

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


%%% Earlier Approach of taking sines and cosines to approximate the
%%% trajectory. This approach causes overshoot at the end because the
%%% velocity is not zero at t_end.

% z = 2.5;
% 
% total_time = 14;
% t_interval = total_time / 3;
% 
% vmax = z / (2 * t_interval);
% amax = vmax / (t_interval);
% 
% tmax = total_time;
% num_circles = 1;
% omega = 2 * pi * num_circles / (tmax);
% A = 5;
% 
% if t <= tmax / 3
%     pos(3) = 0.5 * amax * (t ^ 2);
%     vel(3) = amax * t;
%     acc(3) = amax;
% elseif t <= 2 * tmax / 3
%     deltat = t - tmax/3;
%     pos(3) = (vmax^2) / (2 * amax) + vmax * deltat;
%     vel(3) = vmax;
%     acc(3) = 0;
% elseif t <= tmax
%     deltat = t - 2 * tmax / 3;
%     pos(3) = 3 * (vmax^2) / (2 * amax) + vmax*deltat - 0.5*amax*deltat^2;
%     vel(3) = vmax - amax * deltat;
%     acc(3) = -amax;
% else
%     vel(3) = 0;
%     acc(3) = 0;
%     pos(3) = 2*(vmax^2)/amax;
% end
% 
% if t < tmax
%     pos(1) = A * cos(omega * t);
%     vel(1) = -A * omega * sin(omega * t);
%     acc(1) = -A * omega^2 * cos(omega * t);
%     pos(2) = A * sin(omega * t);
%     vel(2) = A * omega * cos(omega * t);
%     acc(2) = -A * omega^2 * sin(omega * t);
% else
%     vel(1) = 0;
%     acc(1) = 0;
%     vel(2) = 0;
%     acc(2) = 0;
%     pos(1) = A * cos(omega * tmax);
%     pos(2) = A * sin(omega * tmax);
% end
% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
