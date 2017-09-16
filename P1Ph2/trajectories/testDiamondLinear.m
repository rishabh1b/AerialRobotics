%% TestDiamondLinear and Smoothing
t_int = 1;
p  = 1;
tstep = 4*t_int/100;
XYZ = [0 0 0];
for t = tstep:tstep:4*t_int
    [x,y,z] = diamondLinear(t, p, t_int);
    XYZ = [XYZ;[x,y,z]];
end
t = 0:tstep:4*t_int;
figure
subplot(3,2,1)
plot(t,XYZ(:,1),'r')
subplot(3,2,3)
plot(t,XYZ(:,2), 'g')
subplot(3,2,5)
plot(t,XYZ(:,3), 'b')

%
num_points_interval = 10;
order = 7;
X = XYZ(:,1);
Y = XYZ(:,2);
Z = XYZ(:,3);
X_subset = X(1:num_points_interval:end);
Y_subset = Y(1:num_points_interval:end);
Z_subset = Z(1:num_points_interval:end);
t_subset = t(1:num_points_interval:end);
t_subset = t_subset';
p_x = polyfit(t_subset, X_subset, order);
p_y = polyfit(t_subset, Y_subset, order);
p_z = polyfit(t_subset, Z_subset, order);

subplot(3,2,2)
plot(t,polyval(p_x,t),'r')
subplot(3,2,4)
plot(t,polyval(p_y,t),'g')
subplot(3,2,6)
plot(t,polyval(p_z,t),'b')