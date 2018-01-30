function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  an obstacle.

% CMSC 828T Proj 1 Phase 1

% Output structure: 
    % Map is a cell array containing the following:
    %   --> map{1} contains a 3D logical occupancy grid
    %   --> map{2}, Map{3} and Map{4} store the discretized axes values
    %       corresponding to the X, Y and Z axes respectively
    %   --> map{5} and map{6} store the xy_res and z_res respectively


% Open file, read data, close file. Comments in file marked by #
fileID = fopen(filename);
fileDat = textscan(fileID,'%s %f %f %f %f %f %f %f %f %f','CommentStyle','#');
fclose(fileID);

%% START YOUR CODE HERE %%
% Get the Boundary
num_blocks = size(fileDat{1}, 1);

if num_blocks == 0
    map = [];
    return;
end

% Get the Blocks
blocks = zeros(num_blocks-1, 9);
j = 1;
for i = 1:num_blocks
    if size(cell2mat(fileDat{1}(i)),2) > 5 %Boundary
        xmin = fileDat{2}(i);
        ymin = fileDat{3}(i);
        zmin = fileDat{4}(i);

        xmax = fileDat{5}(i);
        ymax = fileDat{6}(i);
        zmax = fileDat{7}(i);
    else
        blocks(j,:) = [fileDat{2}(i) fileDat{3}(i) fileDat{4}(i) fileDat{5}(i) ...
                       fileDat{6}(i) fileDat{7}(i) fileDat{8}(i) fileDat{9}(i) ...
                       fileDat{10}(i)];
        j = j + 1;
    end
end
% Inflate the Blocks
inflated_blocks = blocks;
inflated_blocks(:,1) = max((blocks(:,1) - margin), xmin);
inflated_blocks(:,2) = max((blocks(:,2) - margin), ymin);
inflated_blocks(:,3) = max((blocks(:,3) - margin), zmin);
inflated_blocks(:,4) = min((blocks(:,4) + margin), xmax);
inflated_blocks(:,5) = min((blocks(:,5) + margin), ymax);
inflated_blocks(:,6) = min((blocks(:,6) + margin), zmax);

% Get the grid indices of the vertices of the blocks

% ceil should be used, since value slightly greater than 1 for example 
% lies in the second cell.
grid_index_block = zeros(num_blocks-1, 6);
grid_index_block(:,1) = ceil((inflated_blocks(:,1) - xmin) / xy_res);
grid_index_block(:,2) = ceil((inflated_blocks(:,2) - ymin) / xy_res);
grid_index_block(:,3) = ceil((inflated_blocks(:,3) - zmin) / z_res);

grid_index_block(:,4) = ceil((inflated_blocks(:,4) - xmin) / xy_res);
grid_index_block(:,5) = ceil((inflated_blocks(:,5) - ymin) / xy_res);
grid_index_block(:,6) = ceil((inflated_blocks(:,6) - zmin) / z_res);

grid_index_block(grid_index_block == 0) = 1;

% Create the Occupancy Map
n = floor((xmax - xmin)/xy_res) + 1;
m = floor((ymax - ymin)/xy_res) + 1;
t = floor((zmax - zmin)/z_res) + 1;

% Initialize the Map
map_occu = false(m-1,n-1,t-1);
for i = 1:num_blocks-1
    x_range = grid_index_block(i,1):grid_index_block(i,4);
    % Flip the co-ordinates
    y_range = m - grid_index_block(i,5):m - grid_index_block(i,2);
    z_range = grid_index_block(i,3):grid_index_block(i,6);  
    map_occu(y_range, x_range, z_range) = true;
end

% Return the value
map{1} = map_occu;
map{2} = xmin:xy_res:xmax;
map{3} = ymin:xy_res:ymax;
map{4} = zmin:z_res:zmax;
map{5} = xy_res;
map{6} = z_res;
map{7} = blocks; % For Rendering
map{8} = inflated_blocks; %For Collision checking
%% END YOUR CODE HERE %%
end
