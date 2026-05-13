function [particles] = update_particle_filter(read_only_vars, public_vars)
% Hlavni smycka casticoveho filtru (posun, mereni, vyber)

particles = public_vars.particles;

% 1. Predikce
for i = 1:size(particles, 1)
    particles(i,:) = predict_pose(particles(i,:), public_vars.motion_vector, read_only_vars);
end

% 2.Korekce
measurements = zeros(size(particles, 1), length(read_only_vars.lidar_config));

for i = 1:size(particles, 1)
    measurements(i,:) = compute_lidar_measurement(read_only_vars.map, particles(i,:), read_only_vars.lidar_config);
end

% Porovname simulovany Lidar s tim opravdovym a pridelime vahy
weights = weight_particles(measurements, read_only_vars.lidar_distances);

%3. Resampling
% Rozmnozime castice s vysokou vahou a smazeme ty, co jsou uplne mimo
particles = resample_particles(particles, weights);

end