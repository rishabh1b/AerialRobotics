map = load_map('sample_maps/map4.txt', 0.1, 1, 0.3);
%%
start = [1 1 1];
goal = [5 10 4];
%start = [2 2 2];
%goal = [14 2 2];
tic
[path, numexpanded] = dijkstra(map, start, goal,1);
toc
plot_path(map, path);