function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.

% CMSC 828T Proj 1 Phase 1

% If the map is empty, output empty vector else, compute
if isempty(map)
    C = [];
else
    %% START YOUR CODE HERE %%
xq = points(:,1);
yq = points(:,2);
zq = points(:,3);
blocks = map{8};
num_blocks = size(blocks,1);
n = size(points,1);
C = false(n,1);
for i = 1:n
    for j = 1:num_blocks
        block = blocks(j,:);
        xmin = block(1);
        ymin = block(2);
        zmin = block(3);
        xmax = block(4);
        ymax = block(5);
        zmax = block(6);

        xv = [xmin xmax xmax xmin xmin];
        yv = [ymin ymin ymax ymax ymin];
        in = inpolygon(xq,yq,xv,yv);
        if any(in == 1 & zq >= zmin & zq <= zmax) == true
            C(i) = true;
            break;
        end
    end
end
    %% END YOUR CODE HERE %%
end
end
