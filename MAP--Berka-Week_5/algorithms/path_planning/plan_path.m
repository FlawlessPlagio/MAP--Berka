function [path] = plan_path(read_only_vars, public_vars)
    % Cestu vygenerujeme jen při první iteraci
    if isempty(public_vars.path) 
        
        % TASK 1: "Manually design a trajectory"
        % Bezpečný koridor na y = 8 (mezi spodní a horní překážkou)
        
        % 1. úsek: Ze startu [2,2] vyjedeme čistě nahoru na [2, 8]
        y1 = linspace(2, 8, 20)';
        x1 = ones(20, 1) * 2;
        
        % 2. úsek: Z [2,8] jedeme doprava na [16, 8] (protažení koridorem)
        x2 = linspace(2, 16, 40)';
        y2 = ones(40, 1) * 8;
        
        % 3. úsek: Z [16, 8] sjedeme dolů rovnou do cíle [16, 2]
        y3 = linspace(8, 2, 20)';
        x3 = ones(20, 1) * 16;
        
        % Spojíme všechny úseky do jedné trasy
        path = [x1, y1; x2, y2; x3, y3];
        
    else
        % Pokud už cesta byla vygenerována, jen ji vrátíme zpět
        path = public_vars.path; 
    end
end