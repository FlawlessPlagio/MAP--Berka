function [estimated_pose] = estimate_pose(public_vars)
    % spocita stred mraku castic (toho naseho odhadu)
    if isfield(public_vars, 'particles') && ~isempty(public_vars.particles)
        x_est = mean(public_vars.particles(:, 1));
        y_est = mean(public_vars.particles(:, 2));
        
        % prumer uhlu se musi delat pres sin a cos
        sin_sum = sum(sin(public_vars.particles(:, 3)));
        cos_sum = sum(cos(public_vars.particles(:, 3)));
        theta_est = atan2(sin_sum, cos_sum);
        
        estimated_pose = [x_est, y_est, theta_est];
    else
        estimated_pose = nan(1,3);
    end
end