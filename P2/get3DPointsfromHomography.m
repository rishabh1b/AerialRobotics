function X = get3DPointsfromHomography(H, j, K)
pixel_points = [j(1) j(3) j(5) j(7);...
                j(2) j(4) j(6) j(8)];
% pr = [ 1 0 0;
%        0 1 0 ];
%pixel_points = pixel_points ./ scale;
p_m = ((K \ [pixel_points; ones(1,4)]));
X = H \ p_m;
X = X ./ X(3,:);
end