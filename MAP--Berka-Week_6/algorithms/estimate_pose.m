function [estimated_pose] = estimate_pose(public_vars)
    % Nyní bereme střed robota rovnou z Kalmanova filtru (mu)
    if isfield(public_vars, 'mu') && ~isempty(public_vars.mu)
        estimated_pose = public_vars.mu;
    else
        estimated_pose = nan(1,3);
    end
end