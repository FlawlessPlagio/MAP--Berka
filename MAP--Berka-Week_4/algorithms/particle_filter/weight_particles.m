function [weights] = weight_particles(particle_measurements, lidar_distances)
    N = size(particle_measurements, 1);
    weights = zeros(N, 1);
    
    % parametr pro gaussovku, mozna se pak bude muset zmenit
    sigma = 0.25; 
    
    for i = 1:N
        simulovano = particle_measurements(i, :);
        realita = lidar_distances;
        
        % odchylka a ignorovani NaN (paprsky do blba)
        rozdil = simulovano - realita;
        platne = ~isnan(rozdil);
        
        if any(platne)
            % prumer ctvercu chyb (MSE) jen z tech platnych
            mse = sum(rozdil(platne).^2) / sum(platne);
            weights(i) = exp(-mse / (2 * sigma^2));
        else
            % kdyz je uplne mimo, dame mu aspon malou sanci aby to nespocitalo nulu
            weights(i) = 1e-10; 
        end
    end
    
    % normalizace aby soucet vah byl 1 (pro ten resampling potom)
    suma_vah = sum(weights);
    if suma_vah > 0
        weights = weights / suma_vah;
    else
        % fail-safe kdyby nahodou vsechny umrely
        weights = ones(N, 1) / N; 
    end
end