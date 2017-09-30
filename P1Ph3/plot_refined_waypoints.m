function plot_refined_waypoints(waypoints)
figure(1)
hold on
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), '*', 'MarkerSize', 5, 'LineWidth',2)
end