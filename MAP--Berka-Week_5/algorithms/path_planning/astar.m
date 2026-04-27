function [path] = astar(read_only_vars, public_vars)
%ASTAR - A* algoritmus s nafukovanim zdi

original_grid = read_only_vars.discrete_map.map;
[rows, cols] = size(original_grid);

% zjisteni rozliseni a posunu mapy
res = 0.2; 
if isfield(read_only_vars.discrete_map, 'resolution')
    res = read_only_vars.discrete_map.resolution;
elseif isfield(read_only_vars, 'map_resolution')
    res = read_only_vars.map_resolution;
end

ox = 0; oy = 0;
if isfield(read_only_vars.discrete_map, 'position')
    ox = read_only_vars.discrete_map.position(1);
    oy = read_only_vars.discrete_map.position(2);
end

% nafouknuti prekazek (clearance) pres 2D konvoluci misto imdilate
inflation_px = ceil(0.25 / res); 
kernel = ones(2 * inflation_px + 1); 
grid = conv2(double(original_grid > 0), kernel, 'same') > 0;

% nacteni startu z EKF a cile z read_only (aby to slo vsude)
start_pos = public_vars.mu(1:2); 
if isfield(read_only_vars, 'target_position')
    goal_pos = read_only_vars.target_position(1:2);
elseif isfield(read_only_vars, 'goal_position')
    goal_pos = read_only_vars.goal_position(1:2);
else
    goal_pos = [16, 2]; % fallback
end

% prepocet souradnic na indexy matice
sc = min(cols, max(1, round((start_pos(1) - ox) / res) + 1));
sr = min(rows, max(1, round((start_pos(2) - oy) / res) + 1)); 
gc = min(cols, max(1, round((goal_pos(1) - ox) / res) + 1));
gr = min(rows, max(1, round((goal_pos(2) - oy) / res) + 1)); 

% fallback kdyz nafouknuti spolkne cil nebo start
if grid(sr, sc) > 0 || grid(gr, gc) > 0
    grid = original_grid > 0; 
    if grid(sr, sc) > 0 || grid(gr, gc) > 0
        path = []; return;
    end
end

% inicializace A* struktur
g_score = inf(rows, cols);
f_score = inf(rows, cols);
came_from = zeros(rows, cols);
visited = false(rows, cols);

start_idx = sub2ind([rows, cols], sr, sc);
goal_idx = sub2ind([rows, cols], gr, gc);

g_score(start_idx) = 0;
f_score(start_idx) = norm([sr - gr, sc - gc]);
open_set = start_idx;

% smery a ceny pohybu (8-okoli)
dirs = [0 1; 1 0; 0 -1; -1 0; 1 1; 1 -1; -1 1; -1 -1];
costs = [1, 1, 1, 1, sqrt(2), sqrt(2), sqrt(2), sqrt(2)];

% hlavni cyklus
while ~isempty(open_set)
    [~, min_i] = min(f_score(open_set));
    curr_idx = open_set(min_i);

    % nasli jsme cil
    if curr_idx == goal_idx
        p_idx = curr_idx;
        while p_idx(1) ~= start_idx
            p_idx = [came_from(p_idx(1)), p_idx];
        end
        
        % prevod indexu zpatky na metry
        [pr, pc] = ind2sub([rows, cols], p_idx);
        path_x = (pc - 1) * res + ox;
        path_y = (pr - 1) * res + oy;
        path = [path_x; path_y]';
        return;
    end

    open_set(min_i) = [];
    visited(curr_idx) = true;

    [cr, cc] = ind2sub([rows, cols], curr_idx);

    % pruchod sousedu
    for i = 1:8
        nr = cr + dirs(i, 1);
        nc = cc + dirs(i, 2);

        % kontrola mezi a zdi
        if nr < 1 || nr > rows || nc < 1 || nc > cols || grid(nr, nc) > 0 || visited(nr, nc)
            continue;
        end

        n_idx = sub2ind([rows, cols], nr, nc);
        tentative_g = g_score(curr_idx) + costs(i);

        % update cesty
        if tentative_g < g_score(n_idx)
            came_from(n_idx) = curr_idx;
            g_score(n_idx) = tentative_g;
            h = norm([nr - gr, nc - gc]);
            f_score(n_idx) = tentative_g + h;

            if ~ismember(n_idx, open_set)
                open_set(end + 1) = n_idx;
            end
        end
    end
end

path = [];
end