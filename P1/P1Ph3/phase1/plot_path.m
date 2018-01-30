function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.

% CMSC 828T Proj 1 Phase 1

%% START YOUR CODE HERE %%
blocks = map{7};

n = size(blocks,1);
view(3)
axis vis3d
axis([map{2}(1) map{2}(end) map{3}(1) map{3}(end) map{4}(1) map{4}(end)])
for i = 1:n
   A = blocks(i,:);
   xmin = A(1);
   ymin = A(2);
   zmin = A(3);
   xmax = A(4);
   ymax = A(5);
   zmax = A(6);
   r = (A(7) - 0)/255;
   g = (A(8) - 0) / 255;
   b = (A(9) - 0) / 255;
   
   vertices = [xmin ymin zmin;...
               xmax, ymin, zmin;...
               xmax, ymax, zmin;...
               xmin, ymax, zmin;...
               xmin, ymin, zmax;...
               xmax, ymin, zmax;...
               xmax, ymax, zmax;...
               xmin, ymax, zmax];
  faces = [1 2 6 5;...
           2 3 7 6;...
           3 7 8 4;...
           4 8 5 1;...
           1 2 3 4;...
           5 6 7 8];
  patch('Faces', faces, 'Vertices', vertices, 'FaceColor', [r g b])      
end
hold on
grid on
% Plot Path
plot3(path(:,1), path(:,2), path(:,3),'LineWidth', 0.5, 'LineStyle', '--')
%% END YOUR CODE HERE %%

end