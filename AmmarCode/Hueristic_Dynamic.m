%Huerstic

function est = Hueristic_Dynamic(L1, map)
[X, Y, DX, DY] = dynamic_state_from_index(map, L1);
[X_goal, Y_goal, DX_goal, DY_goal] = dynamic_state_from_index(map, get_goal(map));

% est = euclidian([X,Y], [X_goal, Y_goal]);
est = manhattan([X,Y], [X_goal, Y_goal]);


end


function d = euclidian(L1, goal)
d = norm(L1 - goal);
end

function d = manhattan(L1, goal)
    d = sum(abs(L1-goal));
end