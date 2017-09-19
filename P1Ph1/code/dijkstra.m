function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.

% CMSC 828T Proj 1 Phase 1

if nargin < 4
    astar = 0;
end

%% START YOUR CODE HERE %%
% keep track of the number of nodes that are expanded
num_expanded = 0;
xmin = map{2}(1);
ymin = map{3}(1);
zmin = map{4}(1);
xmax = map{2}(end);
ymax = map{3}(end);
zmax = map{4}(end);

% Catch Obviously wrong start and goal positions
if collide(map, start) || collide(map,goal) || start(1) < xmin || start(1) > xmax || ...
        start(2) < ymin || start(2) > ymax || start(3) < zmin || start(3) > zmax
    path = [];
    return;
end
[m,n,t] = size(map{1});
xy_res = map{5};
z_res = map{6};

% Some Data Structures
graph = zeros(m,n,t);
parentMap = zeros(m,n,t);
distanceFromStart = Inf(m,n,t); % Priority Queue
f = Inf(m,n,t); % Function d(node) + H(node)

start_grid_w = zeros(3,1);
goal_grid_w = zeros(3,1);

% Mapping start and end co-ordinates to the 3D Matrix
start_grid_w(1) = floor((start(1) - xmin) / xy_res);
start_grid_w(2) = floor((start(2) - ymin) / xy_res);
start_grid_w(3) = floor((start(3) - zmin) / z_res);

goal_grid_w(1) = floor((goal(1) - xmin) / xy_res);
goal_grid_w(2) = floor((goal(2) - ymin) / xy_res);
goal_grid_w(3) = floor((goal(3) - zmin) / z_res);

start_grid_w(start_grid_w == 0) = 1;
goal_grid_w(goal_grid_w == 0) = 1;

curr_row = m + 1 - start_grid_w(2);
curr_col = start_grid_w(1);
curr_z = start_grid_w(3);

goal_row = m + 1 - goal_grid_w(2);
goal_col = goal_grid_w(1);
goal_z = goal_grid_w(3);

curr_node = sub2ind([m,n,t], curr_row, curr_col, curr_z);
goal_node = sub2ind([m,n,t], goal_row, goal_col, goal_z);

distanceFromStart(curr_node) = 0;

% Initialize the Neighbours array
neighbours = zeros(26,3);

% Distances for Each Neighbours
dist = zeros(26,1);
dist(1:6) = 1;
dist(7:18) = sqrt(2);
dist(18:end) = sqrt(3);

% Heuristic for each node
[X,Y,Z] = meshgrid(1:n,1:m,1:t);
H = sqrt((X - goal_col).^2 + (Y - goal_row).^2 + (Z - goal_z).^2);
f(curr_node) = astar * H(curr_node);

while true
    [min_v, current] = min(f(:));
    if current == goal_node || isinf(min_v)
        break;
    end
    graph(current) = 1; % Mark as visited
    f(current) = inf; % remove this node from future considerations
    
    % Get the row, col, and z of this node. 
    [row,col,z] = ind2sub([m,n,t], current);
    
    % Get the neighbours, using 26
    neighbours(1,:) = [row, col - 1, z];
    neighbours(2,:) = [row - 1, col, z];
    neighbours(3,:) = [row, col + 1, z];
    neighbours(4,:) = [row + 1, col, z];
    neighbours(5,:) = [row, col, z + 1];
    neighbours(6,:) = [row, col - 1, z - 1];
    neighbours(7,:) = [row - 1, col - 1, z];
    neighbours(8,:) = [row - 1, col + 1, z];
    neighbours(9,:) = [row + 1, col + 1, z];
    neighbours(10,:) = [row + 1, col - 1, z];
    neighbours(11,:) = [row, col - 1, z - 1];
    neighbours(12,:) = [row - 1, col, z - 1];
    neighbours(13,:) = [row, col + 1, z - 1];
    neighbours(14,:) = [row + 1, col, z - 1];
    neighbours(15,:) = [row, col - 1, z + 1];
    neighbours(16,:) = [row - 1, col, z + 1];
    neighbours(17,:) = [row + 1, col, z + 1];
    neighbours(18,:) = [row, col + 1, z + 1];
    neighbours(19,:) = [row - 1, col - 1, z - 1];
    neighbours(20,:) = [row - 1, col + 1, z - 1];
    neighbours(21,:) = [row + 1, col + 1, z - 1];
    neighbours(22,:) = [row + 1, col - 1, z - 1];
    neighbours(23,:) = [row - 1, col - 1, z + 1];
    neighbours(24,:) = [row - 1, col + 1, z + 1];
    neighbours(25,:) = [row + 1, col + 1, z + 1];
    neighbours(26,:) = [row + 1, col - 1, z + 1];
    
    num_expanded = num_expanded + 1;
    % Go over all the neighbours
    for i = 1:26
        if neighbours(i,1) > 0 && neighbours(i,1) <= m &&...
            neighbours(i,2) > 0 && neighbours(i,2) <= n &&...
             neighbours(i,3) > 0 && neighbours(i,3) <= t &&...
             graph(neighbours(i,1), neighbours(i,2), neighbours(i,3)) ~= 1 && ...
             ~map{1}(neighbours(i,1), neighbours(i,2), neighbours(i,3)) && ...
            distanceFromStart(neighbours(i,1), neighbours(i,2), neighbours(i,3)) > min_v + dist(i)
        distanceFromStart(neighbours(i,1), neighbours(i,2), neighbours(i,3)) = min_v + dist(i);
        ind = sub2ind([m,n,t], neighbours(i,1), neighbours(i,2), neighbours(i,3));
        f(ind) = distanceFromStart(neighbours(i,1), neighbours(i,2), neighbours(i,3)) + astar * H(ind);
        parentMap(neighbours(i,1), neighbours(i,2), neighbours(i,3)) = current;
        end
    end
end

if isinf(distanceFromStart(goal_node))
    path = [];
else
    path = [goal(1) goal(2) goal(3)];
    curr_node = goal_node;        
    curr_node_grid_w = zeros(1,3);
    real_world_point = zeros(1,3);
    while parentMap(curr_node) ~= 0
        curr_node = parentMap(curr_node);
        [row,col,z] = ind2sub([m,n,t], curr_node);
        curr_node_grid_w(2) = m + 1 - row;
        curr_node_grid_w(1) = col;
        curr_node_grid_w(3) = z;
        real_world_point(1) = curr_node_grid_w(1) * xy_res + xmin;
        real_world_point(2) = curr_node_grid_w(2) * xy_res + ymin;
        real_world_point(3) = curr_node_grid_w(3) * z_res + zmin;
        path = [real_world_point; path];
    end
    path(1,:) = [start(1) start(2) start(3)];
%% END YOUR CODE HERE %%

end