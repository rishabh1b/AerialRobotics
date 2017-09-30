close all;
clear all;
clc;
addpath(genpath('./'));

%% Plan path
disp('Planning ...');
% map = load_map('maps/map1.txt', 0.1, 2.0, 0.25);
% start = {[0.0  -4.9 0.2]};
% stop  = {[6.0  18.0-6 3.0]};
%stop  = {[6.0  17 4.0]};

% map = load_map('maps/map3.txt', 0.1, 1.0, 0.4);
% start = {[0 2 5]};
% stop  = {[20  3 4]};

%start = {[1.5  2 2]};
%start = {[1.5  2 2]};
%stop  = {[7.8  2 4]};
%stop  = {[17  2 2]};

map = load_map('maps/map2.txt', 0.1, 1.0, 0.2);
start = {[1 -4 1]};
stop  = {[5 25 3]};

nquad = length(start);
for qn = 1:nquad
    path{qn} = dijkstra(map, start{qn}, stop{qn}, true);
end
if nquad == 1
    plot_path(map, path{1});
else
    % you could modify your plot_path to handle cell input for multiple robots
end

%% Additional init script
init_script;

%% Run trajectory
trajectory = test_trajectory(start, stop, map, path, true); % with visualization
