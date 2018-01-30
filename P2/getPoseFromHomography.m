function [R, t] = getPoseFromHomography(H,K)
% Estimate position and orientation with respect to a set of 4 points on
% the tag
% Inputs:
%    H - the computed homography from the corners in the image
%    render_points - size (N x 3) matrix of world points to project
%    K - size (3 x 3) calibration matrix for the camera
% Outputs: 
% The Camera Pose

%% Extract the pose from the homography
B_tild = (K \ H);
if det(B_tild) < 0
    B = -B_tild;
else
    B = B_tild;
end
lambda = inv((norm(K \ H(:,1)) + norm(K \ H(:,2))) / 2);
r1 = lambda.* B(:,1);
r2 = lambda.* B(:,2);
r3 = cross(r1,r2);
R = [r1 r2 r3];
t = lambda .* B(:,3);
end