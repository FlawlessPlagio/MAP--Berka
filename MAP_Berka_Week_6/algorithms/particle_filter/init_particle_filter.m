function [public_vars] = init_particle_filter(read_only_vars, public_vars)
    % TASK 4: Nahodne rozhozeni castic po mape
    % 200 staci na to, aby to udelalo mrak a nezaseklo to pocitac
    N = 800; 
    
    limity = read_only_vars.map.limits;
    xmin = limity(1);
    ymin = limity(2);
    xmax = limity(3);
    ymax = limity(4);
    
    % vygenerovani nahodnych pozic v ramci mapy
    x = xmin + rand(N, 1) * (xmax - xmin);
    y = ymin + rand(N, 1) * (ymax - ymin);
    
    % uhel od -pi do pi
    theta = -pi + rand(N, 1) * 2 * pi;
    
    public_vars.particles = [x, y, theta];
end