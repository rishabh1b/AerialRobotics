map = load_map('sample_maps/map3.txt', 1, 1, 0);
%%
start = [1 1 1];
%goal = [5 10 4];
goal = [5 26 4];
tic
[path, numexpanded] = dijkstra(map, start, goal,1);
toc
plot_path(map, path);