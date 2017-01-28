% A*
addpath('~/Documents/ROB534 - SDM/HW1/2D')
addpath('~/Documents/ROB534 - SDM/HW1/AmmarCode')
addpath('~/Documents/ROB534 - SDM/HW1/pq')
cd('~/Documents/ROB534 - SDM/HW1')

% Constants %
map = read_map('maze1.pgm');
move_cost = 1;

% Initialization %
goal = get_goal(map);
start= get_start(map);
cost_all = 2000*ones(map.R*map.C,1);
cost_all(start) = 0;
goal_flag = 0;
open_list = pq_init(600);
open_list = pq_insert(open_list, start, cost_all(start));
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
plot_path(map, final_path, 'A* with Euclidian Distance Hueristic')
