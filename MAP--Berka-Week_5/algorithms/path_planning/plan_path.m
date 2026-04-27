function [path] = plan_path(read_only_vars, public_vars)
%PLAN_PATH - ridi vypocet trasy

% 1. CEKAME NA INICIALIZACI LOKALIZACE
% Dokud EKF nenasbira data a nevi kde je, nesmime planovat!
if ~isfield(public_vars, 'is_initialized') || ~public_vars.is_initialized
    path = [];
    return;
end

% 2. PLANOVANI (Spusti se az v 51. kroku)
planning_required = isempty(public_vars.path);

if planning_required
    % Nalezeni hrube cesty s odstupem od zdi
    raw_path = astar(read_only_vars, public_vars);
    
    if ~isempty(raw_path)
        % Vyhlazeni cesty 
        path = smooth_path(raw_path);
    else
        path = [];
    end
else
    % pouziti jiz vygenerovane trasy
    path = public_vars.path;
end

end