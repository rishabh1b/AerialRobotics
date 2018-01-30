function X = get3DPointFromH(j, H)
pixelPoints = [j(1) j(3) j(5) j(7);
               j(2) j(4) j(6) j(8)];
pixelPoints  = [pixelPoints;ones(1,4)];
X = H \ pixelPoints;
X = X ./ X(3,:);
end