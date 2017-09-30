function waypoints  = refinePath3(path, map)

lastPoint = path(1,:);
xy_res = map{5};
z_res = map{6};

waypoints = [lastPoint];

firstpoint = path(1,:);
secondpoint = path(2,:);

num_points = size(path,1);
thresh_dist = 3.5;

dir_1 = (secondpoint - firstpoint);
dir_1 = dir_1 ./ norm(dir_1);

i = 2;
found = false;
%skiponce = false;
firstIndexAdded = false;
firstindex = 1;
lastIndexAdded = 1;
while i < num_points
    if i+1 ~= num_points
        dir_2 = path(i+1,:) - path(i,:);
        dir_2 = dir_2 ./ norm(dir_2);
        if abs(dir_2(1) - dir_1(1)) > 0.001 || abs(dir_2(2) - dir_1(2)) > 0.001 || ...
                 abs(dir_2(3) - dir_1(3)) > 0.001
             secondindex = i;
             found = true;
        end
    elseif i == num_points
        secondindex = i;
        found = true;
    else
        secondindex = i + 1;
        found = true;
    end
    
    if found
        chosenIndex = floor((firstindex + secondindex) / 2);
        if chosenIndex ~= firstindex && isCollision(waypoints(end,:),path(chosenIndex,:))
            %waypoints = [waypoints;path(firstindex,:)];
            addWaypoint(firstindex);
        elseif chosenIndex ~= firstindex            
%             waypoints = [waypoints;path(chosenIndex,:)];
              addWaypoint(chosenIndex);
        elseif chosenIndex ~= 1 && ~firstIndexAdded
            waypoints = [waypoints;path(firstindex,:)];
            firstIndexAdded = true;
            lastIndexAdded = firstindex;
        else
            firstIndexAdded = false;
        end
        dir_1 = dir_2;
        firstindex = secondindex;
        found = false;
    end
    i = i + 1;
end
waypoints = [waypoints;path(end,:)];

function addWaypoint(index)
    if distance(path(lastIndexAdded,:), path(index,:)) > thresh_dist
        addOneMoreIndex = floor((lastIndexAdded + index) / 2);
        waypoints = [waypoints;path(addOneMoreIndex,:)];
    end
    waypoints = [waypoints;path(index,:)];
    lastIndexAdded = index;
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

function dist = distance(p1,p2)
        dist = sqrt((p2(1) - p1(1))^2 + (p2(2) - p1(2))^2 + (p2(3) - p1(3))^2);
    end
end