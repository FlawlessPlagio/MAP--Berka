function [public_vars] = plan_motion(read_only_vars, public_vars)
% Hlavni funkce pro pohyb robota - bud hleda zed, nebo jede po care

% Pamatujeme si minule hodnoty pro PD regulator a pocitani rohu
persistent prev_error last_steer wall_lost_counter;
if isempty(prev_error), prev_error = 0; end
if isempty(last_steer), last_steer = 0; end
if isempty(wall_lost_counter), wall_lost_counter = 0; end

max_v = read_only_vars.agent_drive.max_vel;
lidar = read_only_vars.lidar_distances;
num_rays = length(lidar);

% === 1. REZIM PRUZKUMU (Jizda podel zdi, nez se zorientujeme)          ===

if ~isfield(public_vars, 'localization_converged') || ~public_vars.localization_converged
    
    n = num_rays;
    if n < 3, public_vars.motion_vector = [0, 0]; return; end
    
    d = lidar;
    [d_min, idx_min] = min(d);
    dist_right = d_min;
    
    % Pocitadlo pro detekci ztraty zdi (napr. za rohem)
    if dist_right > 1.6
        wall_lost_counter = wall_lost_counter + 1;
    else
        wall_lost_counter = 0; 
    end
    
    % Rozdeleni Lidaru na zony (predek, pravo, levo)
    non_right_mask = true(1,n);
    for offset = -1:1
        idx_excl = mod(idx_min - 1 + offset, n) + 1;
        non_right_mask(idx_excl) = false;
    end
    dist_front = min(d(non_right_mask));
    
    idx_neigh1 = mod(idx_min - 2, n) + 1;
    idx_neigh2 = mod(idx_min,     n) + 1;
    dist_right_front = min(d(idx_neigh1), d(idx_neigh2));
    
    idx_opp = mod(idx_min + round(n/2) - 1, n) + 1;
    dist_left = d(idx_opp);
    
    % --- PD Regulator pro udrzeni vzdalenosti 0.75m od prave zdi ---
    target_dist = 0.75; 
    error = target_dist - dist_right;  
    derivative = error - prev_error;
    
    if error > 0
        Kp = 0.7; 
    else
        Kp = 0.35; 
    end
    Kd = 4.5;       
    
    raw_steer = (Kp * error) + (Kd * derivative);
    smooth_steer = 0.65 * last_steer + 0.35 * raw_steer;
    
    % --- Odpuzovani od leve zdi (at se o ni neotreme) ---
    left_repulse = 0.0;
    if dist_left < 0.65
        left_repulse = min(((0.65 - dist_left) / 0.65)^2 * 0.8, 0.6);
    end
    
    % Vysledne zataceni je kombinace sledovani prave zdi a utikani od leve
    combined_steer = max(min(smooth_steer - left_repulse, 1.0), -1.0);
    
    prev_error = error;
    last_steer = smooth_steer;
    
    % --- Vyhodnoceni situace a nastaveni rychlosti kol ---
    is_front_critical   = dist_front < 0.45;
    is_narrow_corridor  = (dist_left + dist_right) < 1.1;
    is_wall_lost        = dist_right > 1.6;
    
    if is_narrow_corridor && is_front_critical
        v_R = -max_v * 0.55; v_L = -max_v * 0.55; % Couvani v uzke ulicce
        
    elseif is_front_critical
        v_R = max_v * 0.85; v_L = -max_v * 0.35;  % Ostra zatacka pred zdi
        
    elseif is_narrow_corridor
        % Jizda presne uprostred uzke chodby
        center_error = dist_left - dist_right;
        center_steer = max(min((center_error / (dist_left + dist_right + 0.01)) * 1.5, 0.7), -0.7);
        v_speed = max_v * 0.65; 
        v_R = v_speed * (1.0 + center_steer); v_L = v_speed * (1.0 - center_steer);
    
    elseif is_wall_lost
        % Ztratili jsme zed, chvili jedeme rovne, pak zatacime doprava
        if wall_lost_counter < 12
            v_R = max_v * 0.80; v_L = max_v * 0.80; 
        else
            v_R = max_v * 0.55; v_L = max_v * 0.90; 
        end
        
    else
        % Standardni rychla jizda podel zdi
        v_speed = max_v * 0.92 * (1.0 - abs(combined_steer) * 0.2);
        v_R = v_speed * (1.0 + combined_steer); 
        v_L = v_speed * (1.0 - combined_steer);
    end
    
    public_vars.motion_vector = [max(min(v_R, max_v), -max_v), max(min(v_L, max_v), -max_v)];
    return;
end

% === 2. JIZDA PO TRASE (Kdyz uz vime, kam jet)                         ===

persistent last_w;
if isempty(last_w), last_w = 0; end

pose = public_vars.estimated_pose;
path = public_vars.path;

if isempty(pose) || any(isnan(pose)) || isempty(path)
    public_vars.motion_vector = [0,0]; 
    return; 
end

% Najdeme bod na ceste, ktery je nam nejbliz
dist_vec = sqrt((path(:,1)-pose(1)).^2 + (path(:,2)-pose(2)).^2);
[~, closest_idx] = min(dist_vec);

% Zjistime prostredi podle modulu (EKF = venku) a blizkosti prekazek
is_outdoor = isfield(public_vars, 'loc_mode') && strcmp(public_vars.loc_mode, 'EKF');
[d_min, ~] = min(lidar); 

if is_outdoor
    % Jsme venku, ale kontrolujeme, jestli kolem nas nejsou prekazky (treba kameny)
    is_cramped = d_min < 0.9; 
    
    if is_cramped
        % Stisneny prostor venku: koukame tesne pred sebe, jedeme pomaleji a tocime ostre
        idx_steer = min(closest_idx + 3, size(path, 1)); 
        idx_horizon = min(closest_idx + 15, size(path, 1)); 
        v_base = max_v * 0.60; 
        Kp_path = 4.0;       
        deadzone = 0.00;     
        smooth_factor = 0.15; 
        speed_exponent = 6;  
    else
        % Volne prostranstvi venku: divame se daleko dopredu a letime naplno
        idx_steer = min(closest_idx + 10, size(path, 1)); 
        idx_horizon = min(closest_idx + 35, size(path, 1)); 
        v_base = max_v;
        Kp_path = 2.5;
        deadzone = 0.01;
        smooth_factor = 0.4;
        speed_exponent = 15;
    end
else
    % Jsme uvnitr budovy: stredne daleky vyhled a plynule zataceni
    idx_steer = min(closest_idx + 6, size(path, 1)); 
    idx_horizon = min(closest_idx + 20, size(path, 1)); 
    
    v_base = max_v;
    Kp_path = 1.2; 
    deadzone = 0.10; 
    smooth_factor = 0.65;
    speed_exponent = 10;
end

% Body na trase, ke kterym se budeme vztahovat
target_steer = path(idx_steer, :);
target_speed = path(idx_horizon, :);

% Spocitani chyby uhlu pro zataceni a pro brzdeni do zatacky
th_err = atan2(target_steer(2)-pose(2), target_steer(1)-pose(1)) - pose(3);
th_err = atan2(sin(th_err), cos(th_err));

hor_err = atan2(target_speed(2)-pose(2), target_speed(1)-pose(1)) - pose(3);
hor_err = atan2(sin(hor_err), cos(hor_err));

% Pokud je chyba uhlu mala (v deadzone), ignorujeme ji, at se robot neklepe
if abs(th_err) < deadzone
    effective_err = 0; 
else
    effective_err = sign(th_err) * (abs(th_err) - deadzone); 
end

% Nelinearni zataceni (vetsi chyba = mnohem ostrejsi zatoceni)
soft_err = abs(effective_err) * effective_err * 2.0; 

% Dynamicka rychlost (zpomali, kdyz vidi pred sebou ostrou zatacku)
speed_factor = 1.0 - min(abs(hor_err) * 1.5, 0.9)^speed_exponent; 
v = v_base * speed_factor;
v = max(v, max_v * 0.40); % Minimalni rychlost, at se nezastavi

% Spocitani uhlove rychlosti s vyhlazenim
raw_w = Kp_path * soft_err; 
w = (smooth_factor) * last_w + (1 - smooth_factor) * raw_w;
last_w = w;

% Prevod rychlosti na jednotliva kola
L = read_only_vars.agent_drive.interwheel_dist;
v_R = v + (w * L / 2); 
v_L = v - (w * L / 2);

public_vars.motion_vector = [max(min(v_R, max_v), -max_v), max(min(v_L, max_v), -max_v)];
end