function [weights] = weight_particles(particle_measurements, lidar_distances)
% Spocita vahy jednotlivych castic podle toho, jak dobre se shoduji s realnym Lidarem

N = size(particle_measurements, 1);
weights = zeros(N, 1);

% Parametr Gaussovy funkce (tolerance chyb)
sigma = 0.25; 

for i = 1:N
    simulovano = particle_measurements(i, :);
    realita = lidar_distances;
    
    % Spocitame odchylku a ignorujeme NaN (coz jsou paprsky, co nic netrefily)
    rozdil = simulovano - realita;
    platne = ~isnan(rozdil);
    
    if any(platne)
        % Prumer ctvercu chyb (MSE) pocitame jen z platnych paprsku
        mse = sum(rozdil(platne).^2) / sum(platne);
        % Prevod chyby na vahu (cim mensi chyba, tim vetsi vaha)
        weights(i) = exp(-mse / (2 * sigma^2));
    else
        % Kdyz nezbyly zadne platne paprsky, dame minimalni vahu misto nuly
        weights(i) = 1e-10; 
    end
end

% Normalizace vah (soucet vsech vah musi byt 1)
suma_vah = sum(weights);
if suma_vah > 0
    weights = weights / suma_vah;
else
    % Fail-safe: kdyz to nahodou vsechno vyjde jako nula, dame vsem stejnou sanci
    weights = ones(N, 1) / N; 
end

end