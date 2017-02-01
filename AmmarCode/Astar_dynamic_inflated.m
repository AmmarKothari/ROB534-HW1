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
map = read_map_for_dynamics('maze2.pgm');
move_cost = 1;
user_epsilon = 10;
time_limit = 1;

% Initialization %
epsilon = user_epsilon;
tic
while toc < time_limit
    t_start = tic;
    goal = get_goal_dynamic(map);
    [Xg, Yg, dx_g, dy_g] = dynamic_state_from_index(map,goal);
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
    path_array = cell(length(cost_all), 1);
    path_array{start} = start;

    while goal_flag ~= 1 && iter < 25*25*3*3*5
        iter = iter + 1;
        if current_location == goal
            goal_flag = 1;
            break
        end
    %     [Xc, Yc, dx_c, dy_c] = dynamic_state_from_index(map,current_location);
    %     if [Xg-Xc,Yg-Yc] == [1,1]
    %         goal_flag = 1;
    %         break
    %     end

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
    %         [x, y, dx, dy] = dynamic_state_from_index(map, neighbors(i));
    %         test_dynamic_neighbors(map, x, y, dx, dy)
    %         pause(0.01)
            if neighbor_cost < cost_all(neighbors(i))
%                disp('Location Being Checked: ' + string(neighbors(i)) + ', Old Cost: ' + string(cost_all(neighbors(i))) + ', New Cost: ' + string(neighbor_cost))
               cost_all(neighbors(i)) = neighbor_cost; 
               open_list = pq_insert(open_list, neighbors(i), neighbor_cost);
               path_array{neighbors(i)} = [neighbors(i); path_array{current_location}];
            else
%                 disp('Location Being Checked: ' + string(neighbors(i)) + ' had no new neighbors')
                continue
            end
        end


    end
    % need to find path from goal to start
%     current_location = start;
%     while all(current_location ~= goal)
%         [X, Y, dx, dy] = dynamic_state_from_index(map, current_location);
%         final_path = [final_path; X,Y];
% %         disp('dx: ' + string(dx) + ', dy: ' + string(dy))
%         neighbors = get_neighbors_dynamic(map, current_location);
% %         neighbors = get_neighbors_dynamic_rev(map, current_location);
%         neighbors = neighbors((neighbors == current_location)~=1); %removing self from neighbors
%         if isempty(neighbors)
%             hello = 1;
%         end
%         [next_val, next_ind] = min(abs(cost_all(neighbors)));
%         next_loc = neighbors(next_ind);
%         current_location = next_loc;
% %         plot_path(map, final_path, 'blah')
% %         pause(0.01)
%     end
    
    for i = 1:length(path_array{goal})
        [x,y] = dynamic_state_from_index(map, path_array{goal}(i));
       final_path = [final_path; [x, y]];
    end
    path_length = length(final_path);
    disp('Epsilon ' + string(epsilon))
    disp('Path Length: ' + string(path_length))
    disp('Nodes Expanded: ' + string(iter))
    disp('Run Time: ' + string(toc))
    disp('---------')
    plot_path(map, final_path, 'A* with Euclidian Distance Hueristic')
%     saveas(gcf, sprintf('name%d', epsilon), 'png')
    epsilon = epsilon - 0.5*(epsilon - 1);
    if epsilon == 1
        break
    end
    if epsilon < 1.001
        epsilon = 1;
    end
    plot_path(map, final_path, 'A* with ManhattanPlus Distance Hueristic')
    pause(0.01)
end