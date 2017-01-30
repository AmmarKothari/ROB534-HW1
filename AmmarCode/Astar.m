% A*
clear all
close all
fdir = fileparts(which(mfilename));
cd(fdir)
addpath('../2D')
addpath('../AmmarCode')
addpath('../pq')
cd('..')

% Constants %
map = read_map('maze2.pgm');
move_cost = 1;

% Initialization %
goal = get_goal(map);
start= get_start(map);
cost_all = 2000*ones(map.R*map.C,1);
cost_all(start) = 0;
goal_flag = 0;
open_list = pq_init(600);
open_list = pq_insert(open_list, start, cost_all(start));
current_location = start;
iter = 0;
% open_list = pq_insert(open_list, start, 0);
final_path = [];

while goal_flag ~= 1
    iter = iter + 1;
    if current_location == goal
        goal_flag = 1;
    end
    
    try
        [open_list, current_location] = pq_pop(open_list);
    catch error
        disp('Uh Oh: No more nodes in queue!')
        hello = 1;
    end
    neighbors = get_neighbors(map, current_location);
    %last value of neighbors is the current location
    %assign neighbors a new cost = move_cost + hueristic
    current_cost = cost_all(current_location);
    for i = 1:length(neighbors)-1
        neighbor_cost = current_cost + move_cost + Hueristic(neighbors(i), map);
        if neighbor_cost < cost_all(neighbors(i))
           cost_all(neighbors(i)) = neighbor_cost;
           if pq_test(open_list, neighbors(i))
               [x, y] = state_from_index(neighbors(i));
               disp('Point already in Queue')
               disp('ID' + string(neighbors(i)) + ' , ' + string(x) + ' ' + string(y))
           end
           open_list = pq_insert(open_list, neighbors(i), neighbor_cost);
        end
    end
    

end
% need to find path from goal to start
current_location = goal;
while current_location ~= start
    [X, Y] = state_from_index(map, current_location);
    final_path = [final_path; X,Y];
    neighbors = get_neighbors(map, current_location);
    neighbors = neighbors(1:end-1); %removing current location from list
    [next_val, next_ind] = min(cost_all(neighbors));
    next_loc = neighbors(next_ind);
    current_location = next_loc;
    
end
plot_path(map, final_path, 'A* with Manhattan Distance Hueristic')
