function [path] = plan_path(read_only_vars, public_vars)
    % Zde měň číslo (1 = přímka, 2 = kruh, 3 = sinusoida)
    path_type = 2; 
    
    % Cestu stačí vygenerovat jen při první iteraci (když je proměnná prázdná)
    if isempty(public_vars.path) 
        
        switch path_type
            case 1
                % 1. Přímka 
                % Diagonála od startu rovnou do cíle 
                x = linspace(2, 18, 50)'; 
                y = linspace(2, 18, 50)';
                path = [x, y];
                
            case 2
                % 2. Kruh (Circle)
                theta = linspace(0, 2*pi, 100)'; 
                r = 6; 
                cx = 10; cy = 10; 
                x = cx + r * cos(theta);
                y = cy + r * sin(theta);
                path = [x, y];
                
            case 3
                % 3. Sinusovka
                x = linspace(2, 18, 100)';
                amplitude = 2;
                y = 3 + amplitude * sin(x); 
                path = [x, y];
        end
    else
        % Pokud už cesta byla vygenerována, jen ji vrátíme zpět
        path = public_vars.path; 
    end
end

