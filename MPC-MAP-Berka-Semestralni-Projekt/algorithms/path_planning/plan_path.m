function [path] = plan_path(read_only_vars, public_vars)
% Hlavni funkce pro planovani trasy

% 1. Cekame, az bude lokalizace hotova
if ~isfield(public_vars, 'is_initialized') || ~public_vars.is_initialized
    path = [];
    return;
end

planning_required = isempty(public_vars.path);

% --- Dynamicke preplanovani ---
% Kdyz se PF na startu splete kvuli symetrii a po rozjezdu se diky Lidaru 
% srovna jinde, robot by se snazil najet na starou trasu. Tady to hlidame.
if ~planning_required && ~isempty(public_vars.estimated_pose)
    pose = public_vars.estimated_pose;
    old_path = public_vars.path;
    
    % Spocitame vzdalenost robota od nejblizsiho bodu trasy
    dist_to_path = min(sqrt((old_path(:,1) - pose(1)).^2 + (old_path(:,2) - pose(2)).^2));
    
    % Pokud je robot vice nez 40 cm od cesty (PF se opravil), preplanujeme
    if dist_to_path > 0.4
        disp('VAROVANI: Jsem moc daleko od trasy, planuji novou!');
        planning_required = true;
    end
end

% --- 2. Samotny vypocet ---
if planning_required
    % Zavolame nas A* algoritmus
    raw_path = astar(read_only_vars, public_vars);
    
    if ~isempty(raw_path)
        % Vyhladime rohy pro rychlou jizdu (Nitro)
        path = smooth_path(raw_path);
    else
        path = [];
    end
else
    path = public_vars.path;
end

end