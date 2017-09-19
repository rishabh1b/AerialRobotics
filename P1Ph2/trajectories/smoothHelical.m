tmax = 12;
num_circles = 1;
omega = 2 * pi * num_circles / (tmax);
A = 5;
V = [];
V2 = [];
for t = 0 : 0.01: tmax
    v_curr = -A * omega * sin(omega * t);
    v_curr_2 = A * omega * cos(omega * t);
    V = [V;v_curr];
    V2 = [V2;v_curr_2];
end
t = 0:0.01:tmax;
order = 5;
num_points_interval = 1;
V_subset = V(1:num_points_interval:end);
V2_subset = V2(1:num_points_interval:end);
t_subset = t(1:num_points_interval:end);
t_subset = t_subset';
V_subset(1) = 0;
V_subset(end) = 0;
V2_subset(1) = 0;
V2_subset(end) = 0;
p_v = polyfit(t_subset, V_subset, order);
p_v(5) = 0;
p_v_2 = polyfit(t_subset, V2_subset, order);
p_v_2(5) = 0;
p_v_2(6) = 0;
figure
subplot(2,2,1)
plot(t,V,'r')
subplot(2,2,2)
plot(t,polyval(p_v,t),'r')
%%
subplot(2,2,3)
plot(t,V2,'g')
subplot(2,2,4)
plot(t,polyval(p_v_2,t),'g')
