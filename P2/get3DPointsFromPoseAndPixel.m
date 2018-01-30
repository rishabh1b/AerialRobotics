function [X] = get3DPointsFromPoseAndPixel(R, T, pixelpoints, K, scale)
size_pixel_points = 8;
pixel_points = [pixelpoints(1) pixelpoints(3) pixelpoints(5) pixelpoints(7);...
                pixelpoints(2) pixelpoints(4) pixelpoints(6) pixelpoints(8)];
pixel_points = [pixel_points;ones(1,  (size_pixel_points/2))];          
P = K * [eye(3) zeros(3,1)] * [R T;0 0 0 1];
P = P ./ scale;
% b = scale * pixel_points - P(:,4);
b = pixel_points - P(:,4);
A = P(:,1:3);
X = A \ b;
end