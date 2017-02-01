% RRT Implementation

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
goal_samp = 0.05;
step_size = 1;

% Initialization %
goal = get_goal(map);
start= get_start(map);
goal_flag = 0;
iter = 0;
current_location = start;
[X_start, Y_start] = state_from_index(map, start);
[X_goal, Y_goal] = state_from_index(map, goal);
node_list = [X_start, Y_start];
parent_list = [0,0];
sampled_points = [];
sample_flag = 0;
tic
while goal_flag ~= 1 && iter < 100000
    while sample_flag == 0
        if rand(1) < goal_samp
            S_randx = X_goal;
            S_randy = Y_goal;
    %         disp('Goal Sampled')
        else
            S_randx = rand(1,1)* 25;
            S_randy = rand(1,1)* 25;
        end
        if ~check_hit(map, S_randx, S_randy, 0,0)
            sample_flag = 1;
        end
    end
%     if rand(1) < goal_samp
%         S_randx = X_goal;
%         S_randy = Y_goal;
%         disp('Goal Sampled')
%     else
%         S_randx = rand(1,1)* 25;
%         S_randy = rand(1,1)* 25;
%     end
    sample_flag = 0;
    sampled_points = [sampled_points; S_randx, S_randy];
%     KDtree = KDTreeSearcher(node_list, 'Distance', 'euclidean');
    nearest_idx = knnsearch(node_list, [S_randx, S_randy]);
    %pick action that moves toward state
    X_nearest = node_list(nearest_idx,1);
    Y_nearest = node_list(nearest_idx,2);
    dx = abs(S_randx - X_nearest);
    dy = abs(S_randy - Y_nearest);
    R = dx/dy;
    Vy = step_size/((R^2 + 1)^0.5);
    Vx = R*Vy;
    X_new = X_nearest + Vx;
    Y_new = Y_nearest + Vy;
    if ~check_hit(map, X_new, Y_new,0,0)
        %check if it collided
        %add new node to list
        node_list = [node_list; X_new, Y_new];
        parent_list = [parent_list; [X_nearest, Y_nearest]];
        if all([X_new, Y_new] > [24,24])
           disp('Close to Goal!')
           goal_flag = 1;
        end
    end

    
    
    iter = iter + 1;
end

[vals, indx_back] = min(abs(node_list - [X_goal, Y_goal]));
backx = node_list(indx_back(1),1);
backy = node_list(indx_back(1),2);
path = [X_goal, Y_goal];
while ~any([backx, backy] <= [1,1])
   path = [path; backx, backy]; %add back point to list
   % find parent of that point. Should be at same index
   [indx_back, ~] = find(node_list == [backx, backy]);
   
   parentx = parent_list(indx_back(1),1);
   parenty = parent_list(indx_back(1),2);
   backx = parentx;
   backy = parenty;
   
end
hold on
plot_path(map, path, 'RRT w/ Euclidian Nearest Node')
% plot(node_list(:,1), node_list(:, 2), 'bo');
% plot(sampled_points(:,1), sampled_points(:,2), 'ro');

hold off
% %find path
% node_unique = unique(node_list, 'rows');
% % current_location = goal;
% current_location = [X_goal, Y_goal];
% path = [X_goal, Y_goal];
% 
% while current_location ~= start
%     KDtree = KDTreeSearcher(node_unique, 'Distance', 'euclidean');
% %     [X_C, Y_C] = state_from_index(map, current_location);
% %     idx = knnsearch(KDtree, [X_C, Y_C]);
%     idx = knnsearch(KDtree, current_location);
%     X_next = node_unique(idx,1);
%     Y_next = node_unique(idx,2);
% %     current_location = index_from_state(map, X_next, Y_next);
%     current_location = [X_next, Y_next];
%     path = [path; X_next, Y_next];
%     log_array = any(node_unique ~= [X_next, Y_next],2);
%     node_unique = node_unique(log_array, :); %removing point from list
%     
%     
% end
time = toc;
disp('Runtime= ' + string(time))
disp('Path Length = ' + string(length(path)))
% plot_path(map, path, 'RRT w/ Manhattan Nearest Node')
