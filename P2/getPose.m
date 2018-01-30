function [R, t] = getPose(H)
h3 = cross(H(:,1),H(:,2));
noisyRot = [H(:,1), H(:,2), h3];
[U,~,V] = svd(noisyRot);
D = [1 0 0;0 1 0; 0 0 det(U*V)];
R = U * D * V;
t = H(:,3) / norm(H(:,1));
end