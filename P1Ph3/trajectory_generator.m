function [ desired_state ] = trajectory_generator(t, qn, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% map: The map structure returned by your load_map function
% path: This is the path returned by your planner (dijkstra function)
%
% desired_state: Contains all the information that is passed to the
% controller, as in phase 2
%
% It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.
% persistent map0 path0
% map0 = map;
% path0 = path;
persistent waypoints0 traj_time d0 total_time eachPolyTime coeffsX coeffsY coeffsZ map0 path0 
persistent polyorder ordersystem numcoeffs

if nargin > 2
    map0 = map;
    path0 = path;
    averagevelocity = 2;
    waypoints = refinePath3(path0{1}, map0);
    %
    figure(1)
    hold on
    %plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'LineWidth', 3)
    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), '*', 'MarkerSize', 5, 'LineWidth',2)
    %
    d = waypoints(2:end,:) - waypoints(1:end-1,:);
    d0 = (sqrt(d(:,1).^2 + d(:,2).^2 + d(:,3).^2));
    total_dist = sum(d0);
    total_time = total_dist / averagevelocity;
    waypoint_seg_len = sqrt(d0);
    traj_time = cumsum(waypoint_seg_len);
    traj_time = traj_time/traj_time(end);
    traj_time = [0; traj_time]';
    traj_time = traj_time*total_time;   
    eachPolyTime = traj_time(1,2:end) - traj_time(1,1:end-1);
    waypoints0 = waypoints;
    num_waypoints = size(waypoints,1);
    polyorder = 5;
    ordersystem = 3;
    numcoeffs = polyorder + 1;
   [coeffsX,coeffsY,coeffsZ] = getCoeffs(polyorder,ordersystem,num_waypoints - 1,waypoints0, eachPolyTime);
   % Utility function to visualize the polynomial fitted.
   CheckPolynomial(coeffsX,coeffsY, coeffsZ, numcoeffs, num_waypoints - 1);
else
    if(t >= total_time)
        last_point = waypoints0(end,:);
        desired_state.pos = last_point';
        desired_state.vel = [0;0;0];
        desired_state.acc = [0;0;0];    
    else
        t_index = find(traj_time > t,1) - 1; 
        if (t_index == 0) 
            t_index = 1; 
        end

        scale = (t-traj_time(t_index)) / eachPolyTime(t_index);
        desired_state.pos = zeros(3,1);
        desired_state.vel = zeros(3,1);
        desired_state.acc = zeros(3,1);
        lowPos = 1 + (polyorder + 1) * (t_index-1);
        highPos = (polyorder + 1) * (t_index);

        %Simple matrix product between Transposes of the two matrices could have done the same thing. 
        desired_state.pos = [sum(coeffsX(lowPos:highPos,1) .* (DerivativeCoefficents(numcoeffs,0,scale))');...
                            sum(coeffsY(lowPos:highPos,1) .* (DerivativeCoefficents(numcoeffs,0,scale))');...
                            sum(coeffsZ(lowPos:highPos,1) .* (DerivativeCoefficents(numcoeffs,0,scale))');...
                            ];
        desired_state.vel = [sum(coeffsX(lowPos:highPos,1) .* (DerivativeCoefficents(numcoeffs,1,scale))');...
                            sum(coeffsY(lowPos:highPos,1) .* (DerivativeCoefficents(numcoeffs,1,scale))');...
                            sum(coeffsZ(lowPos:highPos,1) .* (DerivativeCoefficents(numcoeffs,1,scale))');...
                             ] ./ eachPolyTime(t_index);
        desired_state.acc = [sum(coeffsX(lowPos:highPos,1) .* (DerivativeCoefficents(numcoeffs,2,scale))');...
                            sum(coeffsY(lowPos:highPos,1) .* (DerivativeCoefficents(numcoeffs,2,scale))');...
                            sum(coeffsZ(lowPos:highPos,1) .* (DerivativeCoefficents(numcoeffs,2,scale))');...
                             ] ./ ((eachPolyTime(t_index)) ^ 2);
    end
                     
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end

