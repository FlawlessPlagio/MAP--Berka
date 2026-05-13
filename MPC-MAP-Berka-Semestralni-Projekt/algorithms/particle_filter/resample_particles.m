function [new_particles] = resample_particles(particles, weights)
    N = size(particles, 1);
    new_particles = zeros(size(particles)); % priprava matice pro novou generaci
    
    % udelame si ruletu pres kumulativni soucet vah
    ruleta = cumsum(weights);
    
    for i = 1:N
        %nahodne cislo 0 az 1
        kulicka = rand();
        
        % najdeme prvni index, kde je kumulativni vaha vetsi nez nase kulicka
        idx = find(ruleta >= kulicka, 1, 'first');
        
        % kdyby to nahodou nenaslo nic (treba kvuli zaokrouhlovani)
        if isempty(idx)
            idx = 1; 
        end
        
        % zkopirujeme tu co prezila do novy matice
        new_particles(i, :) = particles(idx, :);
    end
end