%Huerstic

function est = Hueristic_Dynamic(L1, map)
[X, Y, DX, DY] = dynamic_state_from_index(map, L1);
[X_goal, Y_goal, DX_goal, DY_goal] = dynamic_state_from_index(map, get_goal(map));

% est = euclidean([X,Y], [X_goal, Y_goal]);
% est = manhattan([X,Y], [X_goal, Y_goal]);
est = manhattan_dynamic([X,Y], [DX, DY], [X_goal, Y_goal]);
% est = euclidean_dynamic([X,Y], [DX, DY], [X_goal, Y_goal]);
end

function d = manhattan_dynamic(L1, V1, goal)
%incetive to go as fast as possible
    d = manhattan(L1, goal)/4 - sum(V1);
    d = max(0, d); %make sure hueristic can't be negative so weird stuff doens't start to happen
    
end

function d = euclidean_dynamic(L1, V1, goal)
%incetive to go as fast as possible
    d = euclidean(L1, goal)/4 - sum(V1);
    d = max(0, d); %make sure hueristic can't be negative so weird stuff doens't start to happen
    
end

function d = euclidean(L1, goal)
d = norm(L1 - goal);
end

function d = manhattan(L1, goal)
    d = sum(abs(L1-goal));
end