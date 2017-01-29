% RRT Implementation

addpath('~/Documents/ROB534 - SDM/HW1/2D')
addpath('~/Documents/ROB534 - SDM/HW1/AmmarCode')
addpath('~/Documents/ROB534 - SDM/HW1/pq')
cd('~/Documents/ROB534 - SDM/HW1')

% Constants %
map = read_map('maze2.pgm');
move_cost = 1;
goal_samp = 0.05;

% Initialization %
goal = get_goal(map);
start= get_start(map);
goal_flag = 0;
iter = 0;
current_location = start;
[X_start, Y_start] = state_from_index(map, start);
[X_goal, Y_goal] = state_from_index(map, goal);
node_list = [X_start, Y_start];
tic
while goal_flag ~= 1 && iter < 10000
    if rand(1) < goal_samp
        S_rand = goal;
%         disp('Goal Sampled')
    else
        S_rand = randi([1,map.C*map.R]); %pick a point anywhere in the list
    end
    [X_S, Y_S] = state_from_index(map, S_rand);
%     Y_rand = randi([1,map.R]);
    KDtree = KDTreeSearcher(node_list, 'Distance', 'cityblock');
    nearest_idx = knnsearch(KDtree, [X_S, Y_S]);
    %pick action that moves toward state
    X_nearest = node_list(nearest_idx,1);
    Y_nearest = node_list(nearest_idx,2);
    dx = abs(X_S - X_nearest);
    dy = abs(Y_S - Y_nearest);
    if dx > dy
        X_new = X_nearest + sign(dx) * 1;
        Y_new = Y_nearest;
    else
        X_new = X_nearest;
        Y_new = Y_nearest + sign(dy) * 1;
    end
    if ~check_hit(map, X_new, Y_new,0,0)
        %check if it collided
        %add new node to list
        node_list = [node_list; X_new, Y_new];
    end
    
    
%     S_nearest = index_from_state(map, X_nearest, Y_nearest);
%      dist_to_goal = Hueristic(S_nearest, map);
    dist_list = abs(node_list - [X_goal, Y_goal]);
    if any(dist_list == [1,1])
       disp('Close to Goal!')
       goal_flag = 1;
    end

    
    
    iter = iter + 1;
end

%find path
node_unique = unique(node_list, 'rows');
current_location = goal;
path = [X_goal, Y_goal];

while current_location ~= start
    KDtree = KDTreeSearcher(node_unique);
    [X_C, Y_C] = state_from_index(map, current_location);
    idx = knnsearch(KDtree, [X_C, Y_C]);
    X_next = node_unique(idx,1);
    Y_next = node_unique(idx,2);
    current_location = index_from_state(map, X_next, Y_next);
    path = [path; X_next, Y_next];
    log_array = any(node_unique ~= [X_next, Y_next],2);
    node_unique = node_unique(log_array, :); %removing point from list
    
    
end
time = toc;
disp('Runtime= ' + string(time))
disp('Path Length = ' + string(length(path)))
plot_path(map, path, 'RRT w/ Manhattan Nearest Node')
