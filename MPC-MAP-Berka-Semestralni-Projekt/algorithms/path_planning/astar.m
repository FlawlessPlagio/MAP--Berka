function [path] = astar(read_only_vars, public_vars)
% ASTAR - Hledani cesty s udrzovanim se uprostred chodby a vyhlazenim

original_grid = read_only_vars.discrete_map.map;
[rows, cols] = size(original_grid);

% Zjisteni rozliseni a posunu
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

% --- 1. Rucni vypocet vzdalenosti od zdi (nahrada za bwdist z toolboxu) ---
dist_map_px = inf(rows, cols);
[obs_r, obs_c] = find(original_grid > 0);
for i = 1:length(obs_r)
    dist_map_px(obs_r(i), obs_c(i)) = 0;
end

% Projiti mapy tam a zpet pro odhad vzdalenosti (Brushfire)
for r = 2:rows-1
    for c = 2:cols-1
        dist_map_px(r,c) = min([dist_map_px(r,c), dist_map_px(r-1,c)+1, dist_map_px(r,c-1)+1]);
    end
end
for r = rows-1:-1:2
    for c = cols-1:-1:2
        dist_map_px(r,c) = min([dist_map_px(r,c), dist_map_px(r+1,c)+1, dist_map_px(r,c+1)+1]);
    end
end
dist_map_m = dist_map_px * res;

% --- 2. Tvrde zdi (nafouknuti kvuli velikosti robota) ---
inflation_hard = max(1, round(0.2 / res)); 
grid_hard = conv2(double(original_grid > 0), ones(2 * inflation_hard + 1), 'same') > 0;

% --- 3. Nacteni cile ---
start_pos = public_vars.mu(1:2); 
if isfield(read_only_vars, 'target_position')
    goal_pos = read_only_vars.target_position(1:2);
elseif isfield(read_only_vars, 'target')
    goal_pos = read_only_vars.target(1:2);
elseif isfield(read_only_vars, 'goal')
    goal_pos = read_only_vars.goal(1:2);
elseif isfield(read_only_vars, 'map') && isfield(read_only_vars.map, 'target')
    goal_pos = read_only_vars.map.target(1:2);
elseif isfield(read_only_vars, 'map') && isfield(read_only_vars.map, 'goal')
    goal_pos = read_only_vars.map.goal(1:2);
else
    disp('A* VAROVANI: Nenasel jsem CIL! Davam fallback.');
    goal_pos = [16, 2];
end

% Prepocet souradnic na indexy mrizky
sc = min(cols, max(1, round((start_pos(1) - ox) / res) + 1));
sr = min(rows, max(1, round((start_pos(2) - oy) / res) + 1)); 
gc = min(cols, max(1, round((goal_pos(1) - ox) / res) + 1));
gr = min(rows, max(1, round((goal_pos(2) - oy) / res) + 1)); 

% Kdyz se omylem spawneme ve zdi, najdeme nejblizsi volne misto
if grid_hard(sr, sc) > 0
    [fr, fc] = find(grid_hard == 0);
    [~, m_idx] = min((fr - sr).^2 + (fc - sc).^2);
    sr = fr(m_idx); sc = fc(m_idx);
end
if grid_hard(gr, gc) > 0
    [fr, fc] = find(grid_hard == 0);
    [~, m_idx] = min((fr - gr).^2 + (fc - gc).^2);
    gr = fr(m_idx); gc = fc(m_idx);
end

% --- 4. Priprava A* algoritmu ---
g_score = inf(rows, cols);
f_score = inf(rows, cols);
came_from = zeros(rows, cols);
visited = false(rows, cols);

start_idx = sub2ind([rows, cols], sr, sc);
goal_idx = sub2ind([rows, cols], gr, gc);
g_score(start_idx) = 0;
f_score(start_idx) = norm([sr - gr, sc - gc]);
open_set = start_idx;

% 8 smeru pohybu a jejich ceny
dirs = [0 1; 1 0; 0 -1; -1 0; 1 1; 1 -1; -1 1; -1 -1];
costs = [1, 1, 1, 1, sqrt(2), sqrt(2), sqrt(2), sqrt(2)];

% --- 5. Hlavni cyklus ---
while ~isempty(open_set)
    [~, min_i] = min(f_score(open_set));
    curr_idx = open_set(min_i);
    
    % Nasli jsme cil
    if curr_idx == goal_idx
        % Zpetna rekonstrukce cesty
        p_idx = curr_idx;
        while p_idx(1) ~= start_idx
            p_idx = [came_from(p_idx(1)), p_idx];
        end
        [pr, pc] = ind2sub([rows, cols], p_idx);
        path_x = (pc - 1) * res + ox;
        path_y = (pr - 1) * res + oy;
        
        % Vyhlazeni cesty (pridani bodu navic), aby to robota pri jizde necukalo
        if length(path_x) > 1
            t = 1:length(path_x);
            t_fine = 1:0.2:length(path_x);
            path_x_fine = interp1(t, path_x, t_fine, 'linear');
            path_y_fine = interp1(t, path_y, t_fine, 'linear');
            path = [path_x_fine; path_y_fine]';
        else
            path = [path_x; path_y]';
        end
        return;
    end
    
    open_set(min_i) = [];
    visited(curr_idx) = true;
    [cr, cc] = ind2sub([rows, cols], curr_idx);
    
    % Projdeme vsechny sousedy
    for i = 1:8
        nr = cr + dirs(i, 1); nc = cc + dirs(i, 2);
        
        % Kontrola hranic a narazu do zdi
        if nr < 1 || nr > rows || nc < 1 || nc > cols || grid_hard(nr, nc) > 0 || visited(nr, nc)
            continue;
        end
        
        n_idx = sub2ind([rows, cols], nr, nc);
        
        % Penalizace blizkosti zdi - nuti to robota jet stredem chodby
        dist_to_wall = dist_map_m(nr, nc);
        penalty = 0;
        if dist_to_wall < 0.8
            penalty = (0.8 - dist_to_wall)^2 * 20; 
        end
        
        tentative_g = g_score(curr_idx) + costs(i) + penalty;
        
        % Ulozeni lepsi cesty
        if tentative_g < g_score(n_idx)
            came_from(n_idx) = curr_idx;
            g_score(n_idx) = tentative_g;
            f_score(n_idx) = tentative_g + norm([nr - gr, nc - gc]);
            if ~ismember(n_idx, open_set)
                open_set(end + 1) = n_idx; 
            end
        end
    end
end
path = [];
end