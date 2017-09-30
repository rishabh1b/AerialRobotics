function waypoints  = refinePath(path, map)

lastPoint = path(1,:);
xy_res = map{5};
z_res = map{6};

waypoints = [lastPoint];

%threshdist = sqrt(5);
threshdist = 100;
[m,~] = size(path);
for i = 2:m-1
    if (~isCollision(lastPoint, path(i,:)) && isCollision(lastPoint, path(i+1,:))) || ...
            distance(lastPoint, path(i,:)) > threshdist
        waypoints = [waypoints; path(i,:)];
        lastPoint = path(i,:);
    end
end

waypoints = [waypoints;path(end,:)];

    function dist = distance(p1,p2)
        dist = sqrt((p2(1) - p1(1))^2 + (p2(2) - p1(2))^2 + (p2(3) - p1(3))^2);
    end
    function flag = isCollision(firstPoint, endPoint)
        xy_dist = sqrt((firstPoint(1) - endPoint(1))^2 + (firstPoint(2) - endPoint(2))^2);
        z_dist = abs(endPoint(3) - firstPoint(3));
        numPoints = ceil(max(xy_dist/xy_res, z_dist / z_res));
        pointsToBeTested = [linspace(firstPoint(1), endPoint(1),numPoints);
                            linspace(firstPoint(2), endPoint(2),numPoints);
                            linspace(firstPoint(3), endPoint(3),numPoints);];
        flag = any(collide(map,pointsToBeTested') == true);
    end
end