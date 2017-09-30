function [traj_time, total_time] = getTrajTime(waypoints, averagespeed)
d = waypoints(2:end,:) - waypoints(1:end-1,:);
d0 = (sqrt(d(:,1).^2 + d(:,2).^2 + d(:,3).^2));
total_dist = sum(d0);
total_time = total_dist / averagespeed;
waypoint_seg_len = sqrt(d0);
traj_time = cumsum(waypoint_seg_len);
traj_time = traj_time/traj_time(end);
traj_time = [0; traj_time]';
traj_time = traj_time*total_time;  
end