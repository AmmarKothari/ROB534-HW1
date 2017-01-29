% A*
clear all
close all
fdir = fileparts(which(mfilename));
cd(fdir)
addpath('../4D')
addpath('../AmmarCode')
addpath('../pq')
cd('..')

% Constants %
map = read_map_for_dynamics('maze1.pgm');
move_cost = 1;

% Initialization %
goal = get_goal_dynamic(map);
[start, num_nodes] = get_start_dynamic(map);
current_location = start;
maxV = map.maxV;
initial_cost = 20000;
cost_all = initial_cost*ones(num_nodes,1);
cost_all(start) = 0;
goal_flag = 0;
open_list = pq_init(1000);
open_list = pq_insert(open_list, start, cost_all(start));
iter = 0;
final_path = [];

while goal_flag ~= 1
    iter = iter + 1;
    if current_location == goal
        goal_flag = 1;
        break
    end
    
    try
        [open_list, current_location] = pq_pop(open_list);
    catch error
        hello = 1;
    end
    [neighbors, num_neighbors] = get_neighbors_dynamic(map, current_location);
    neighbors = neighbors((neighbors == current_location)~=1); %removing self from neighbors
    current_cost = cost_all(current_location);
    %if a location has no neighbors (a corner) then assign a high cost
    if isempty(neighbors)
        cost_all(current_location) = -initial_cost;
    end
    %last value of neighbors is the current location
    %assign neighbors a new cost = move_cost + hueristic
    for i = 1:length(neighbors)
        neighbor_cost = current_cost + move_cost + Hueristic_Dynamic(neighbors(i), map);
        if neighbor_cost < cost_all(neighbors(i))
           disp('Location Being Checked: ' + string(neighbors(i)) + ', Old Cost: ' + string(cost_all(neighbors(i))) + ', New Cost: ' + string(neighbor_cost))
           cost_all(neighbors(i)) = neighbor_cost; 
           open_list = pq_insert(open_list, neighbors(i), neighbor_cost);
        else
            disp('Location Being Checked: ' + string(neighbors(i)) + ' had no new neighbors')
        end
    end
    

end
% need to find path from goal to start
current_location = start;
while all(current_location ~= goal)
    [X, Y, dx, dy] = dynamic_state_from_index(map, current_location);
    final_path = [final_path; X,Y];
    neighbors = get_neighbors_dynamic(map, current_location);
    neighbors = neighbors((neighbors == current_location)~=1); %removing self from neighbors
    if isempty(neighbors)
        hello = 1;
    end
    [next_val, next_ind] = min(abs(cost_all(neighbors)));
    next_loc = neighbors(next_ind);
    current_location = next_loc;
    
end
plot_path(map, final_path, 'A* with ManhattanPlus Distance Hueristic')
