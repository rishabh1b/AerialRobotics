function waypoints  = refinePath2(path, map)

lastPoint = path(1,:);
xy_res = map{5};
z_res = map{6};

waypoints = [lastPoint];

firstpoint = path(1,:);
secondpoint = path(2,:);

num_points = size(path,1);

dir_1 = (secondpoint - firstpoint);
dir_1 = dir_1 ./ norm(dir_1);

i = 2;
found = false;
%skiponce = false;
firstindex = 1;
while i < num_points
    if i+1 ~= num_points && i ~= num_points
        dir_2 = path(i+1,:) - path(i,:);
        dir_2 = dir_2 ./ norm(dir_2);
        if abs(dir_2(1) - dir_1(1)) > 0.001 || abs(dir_2(2) - dir_1(2)) > 0.001 || ...
                 abs(dir_2(3) - dir_1(3)) > 0.001
             %secondpoint = path(i,:);
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
        if chosenIndex ~= firstindex
            waypoints = [waypoints;path(chosenIndex,:)];
        end
        dir_1 = dir_2;
        firstindex = secondindex;
    end
    i = i + 1;
end
waypoints = [waypoints;path(end,:)];
end