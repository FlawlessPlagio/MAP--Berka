function [public_vars] = plan_motion(read_only_vars, public_vars)
    % 1. Načtení polohy z MoCap
    pose = read_only_vars.mocap_pose;
    
    % Ošetření pro začátek
    if isempty(pose) 
        public_vars.motion_vector = [0, 0];
        return;
    end
    
    x = pose(1); 
    y = pose(2); 
    theta = pose(3); % Natočení robota
    
    % 2. Získání cesty
    path = public_vars.path;
    if isempty(path)
        public_vars.motion_vector = [0, 0];
        return;
    end
    
    % Najdeme nejbližší bod na cestě
    distances = sqrt((path(:,1) - x).^2 + (path(:,2) - y).^2);
    [min_dist, closest_idx] = min(distances);
    
    %Lookahead
    % Kolik bodů cesty dopředu? 
    lookahead_offset = 10; 
    
    lookahead_idx = closest_idx + lookahead_offset; 
    if lookahead_idx > size(path, 1)
        lookahead_idx = size(path, 1); % Ochrana proti přetečení pole
    end
    
    target_x = path(lookahead_idx, 1);
    target_y = path(lookahead_idx, 2);
    
    % 3. Výpočet úhlové chyby
    dx = target_x - x;
    dy = target_y - y;
    
    angle_to_target = atan2(dy, dx);
    heading_error = angle_to_target - theta;
    
    % Normalizace úhlu (aby robot netočil o 360 stupňů navíc)
    heading_error = atan2(sin(heading_error), cos(heading_error));
    
    %Zesílení regulátoru (Kp)
    Kp = 35; 
    
    % 4. Řízení rychlostí
    v = 1; % Lineární rychlost
    
    %zastav
    if closest_idx == size(path, 1) && min_dist < 0.2
        v = 0;
    end
    
    % Výpočet rotační rychlosti
    omega = Kp * heading_error; 
    
    % 5.převod na otáčky levého a pravého kola
    L = read_only_vars.agent_drive.interwheel_dist; 
    
    v_R = v + (omega * L / 2);
    v_L = v - (omega * L / 2);
    
    % Omezení rychlostí
    max_v = read_only_vars.agent_drive.max_vel;
    public_vars.motion_vector = [max(min(v_R, max_v), -max_v), max(min(v_L, max_v), -max_v)];
end