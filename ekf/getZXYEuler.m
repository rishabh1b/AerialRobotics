function eul = getZXYEuler(rot)
if rot(3,2) < 1
    if rot(2,2) > -1
        thetaX = asin(rot(3,2));
        thetaZ = atan2(-rot(1,2), rot(2,2));
        thetaY = atan2(-rot(3,1), rot(2,2));
    else
        % Not a Unique solution
        thetaX = -pi/2;
        thetaZ = -atan2(rot(1,3),rot(1,1));
        thetaY = 0;
    end
else
    % Not Unique
    thetaX = pi/2;
    thetaZ = atan2(rot(1,3),rot(1,1));
    thetaY = 0;
end
eul = [thetaX, thetaY, thetaZ];
end