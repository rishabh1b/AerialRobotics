map = load_map('sample_maps/map1.txt', 0.1, 1, 0.1);
%%
start = [0 -5 0];
goal = [5 0 1];
%start = [2 2 2];
%goal = [14 2 2];
tic
[path, numexpanded] = dijkstra(map, start, goal);
toc
%% Plot?
plot_path(map, path);