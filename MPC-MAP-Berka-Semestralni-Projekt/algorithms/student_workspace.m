function [public_vars] = student_workspace(read_only_vars, public_vars)
% Hlavni ridici smycka - hybridni lokalizace (PF + EKF) s radialnim clusteringem

if (read_only_vars.counter == 1)
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars.gnss_history = [];
    public_vars.is_initialized = false;
    public_vars.localization_converged = false;
    public_vars.gnss_stability_counter = 0; 
    public_vars.kf.Q = zeros(2,2);
    public_vars.mu = [2, 2, pi/2];
    public_vars.sigma = zeros(3,3);
    public_vars.loc_mode = 'PF';
end

% --- 1. Ulozeni GNSS dat a pocitani stability signalu ---
has_gnss = ~isempty(read_only_vars.gnss_position) && ~any(isnan(read_only_vars.gnss_position));
if has_gnss
    public_vars.gnss_history = [public_vars.gnss_history; read_only_vars.gnss_position];
    public_vars.gnss_stability_counter = min(public_vars.gnss_stability_counter + 1, 15);
else
    public_vars.gnss_stability_counter = max(public_vars.gnss_stability_counter - 1, 0);
end

% --- 2. Faze pruzkomu - hledame, kde jsme (Particle Filter) ---
if ~public_vars.localization_converged
    
    public_vars.particles = update_particle_filter(read_only_vars, public_vars);
    
    lims = read_only_vars.map.limits;
    public_vars.particles(:,1) = max(lims(1), min(lims(3), public_vars.particles(:,1)));
    public_vars.particles(:,2) = max(lims(2), min(lims(4), public_vars.particles(:,2)));
    
    % 10% castic rozhazime nahodne po mape jako pruzkumniky, at neuvizneme v lokalnim minimu
    N_scouts = round(size(public_vars.particles, 1) * 0.10);
    scout_idx = randperm(size(public_vars.particles, 1), N_scouts);
    public_vars.particles(scout_idx, 1) = lims(1) + rand(N_scouts, 1) * (lims(3) - lims(1));
    public_vars.particles(scout_idx, 2) = lims(2) + rand(N_scouts, 1) * (lims(4) - lims(2));
    public_vars.particles(scout_idx, 3) = -pi + rand(N_scouts, 1) * 2 * pi;
    
    % --- Vyhodnoceni shody castic (Radialni metoda) ---
    
    % Nejprve najdeme priblizny stred nejvetsiho shluku
    rx = round(public_vars.particles(:,1)); 
    ry = round(public_vars.particles(:,2));
    [~,~,ic] = unique([rx, ry], 'rows');
    mode_idx = mode(ic);
    approx_center = mean(public_vars.particles(ic == mode_idx, 1:2), 1);
    
    % Spocitame vzdalenost vsech castic od tohoto stredu
    dists = sqrt((public_vars.particles(:,1) - approx_center(1)).^2 + ...
                 (public_vars.particles(:,2) - approx_center(2)).^2);
             
    % Hlavni shluk tvori vsechny castice v okruhu 0.8m
    main_cluster_count = sum(dists < 0.8);
    max_cluster_ratio = main_cluster_count / size(public_vars.particles, 1);
    
    % Kontrola symetrie mapy - jestli neni jinde podobny shluk (dal nez 2 metry)
    far_particles = public_vars.particles(dists > 2.0, 1:2);
    if ~isempty(far_particles)
        rx_far = round(far_particles(:,1));
        ry_far = round(far_particles(:,2));
        [~,~,ic_far] = unique([rx_far, ry_far], 'rows');
        second_cluster_count = max(accumarray(ic_far, 1));
    else
        second_cluster_count = 0;
    end
    
    % Pro jistotu chceme, aby hlavni shluk byl aspon 3x vetsi nez ten druhy
    is_unambiguous = main_cluster_count > (second_cluster_count * 3.0);
    
    % Kdyz mame 75% castic pohromade a neni tu symetrie, mame lokalizovano
    is_confident = (max_cluster_ratio > 0.75) && is_unambiguous;
    has_explored_enough = read_only_vars.counter > 100; % Nechame ho jet aspon 100 kroku
    
    is_outdoor_start = public_vars.gnss_stability_counter >= 10;
    
    if is_outdoor_start || (is_confident && has_explored_enough) || read_only_vars.counter > 500
        disp(['Lokalizace USPESNA v kroku: ', num2str(read_only_vars.counter)]);
        public_vars.localization_converged = true;
        
        if is_outdoor_start
            public_vars.loc_mode = 'EKF';
            gnss_cov = cov(public_vars.gnss_history);
            gnss_mean = mean(public_vars.gnss_history);
            public_vars.kf.Q = gnss_cov;
            public_vars.mu = [gnss_mean(1), gnss_mean(2), 0]; 
        else
            public_vars.loc_mode = 'PF';
            % Presny stred pro inicializaci pocitame jen z castic v nasem kruhu
            best_particles = public_vars.particles(dists < 0.8, :);
            public_vars.mu = [mean(best_particles(:,1)), mean(best_particles(:,2)), ...
                              atan2(mean(sin(best_particles(:,3))), mean(cos(best_particles(:,3))))];
        end
        public_vars.is_initialized = true;
    end
    public_vars.estimated_pose = nan(1,3);

% --- 3. Faze jizdy - stridame EKF (venku) a PF (vevnitr) ---
elseif public_vars.is_initialized
    
    [ekf_mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
    public_vars.particles = update_particle_filter(read_only_vars, public_vars);
    
    % Hystereze pro prepinani rezimu (zabrani blikani na hrane budovy)
    if public_vars.gnss_stability_counter >= 10
        public_vars.loc_mode = 'EKF';
    elseif public_vars.gnss_stability_counter <= 2
        public_vars.loc_mode = 'PF';
    end
    
    if strcmp(public_vars.loc_mode, 'EKF')
        public_vars.mu = ekf_mu;
        
        % Udrzujeme castice u EKF odhadu, aby nezabloudily, nez vjedeme do budovy
        N_sync = round(size(public_vars.particles, 1) * 0.8);
        sync_idx = randperm(size(public_vars.particles, 1), N_sync);
        public_vars.particles(sync_idx, 1) = ekf_mu(1) + randn(N_sync, 1) * 0.2;
        public_vars.particles(sync_idx, 2) = ekf_mu(2) + randn(N_sync, 1) * 0.2;
        public_vars.particles(sync_idx, 3) = ekf_mu(3) + randn(N_sync, 1) * 0.1;
    else
        % Ve vnitrnim rezimu pouzijeme opet radialni sken
        approx_center = public_vars.mu(1:2);
        dists = sqrt((public_vars.particles(:,1) - approx_center(1)).^2 + ...
                     (public_vars.particles(:,2) - approx_center(2)).^2);
        best_particles = public_vars.particles(dists < 1.0, :);
        
        if ~isempty(best_particles)
            public_vars.mu = [mean(best_particles(:,1)), mean(best_particles(:,2)), ...
                              atan2(mean(sin(best_particles(:,3))), mean(cos(best_particles(:,3))))];
        end
    end
    public_vars.estimated_pose = public_vars.mu;
end

% --- 4. Planovani trasy a pohybu ---
public_vars.path = plan_path(read_only_vars, public_vars);
public_vars = plan_motion(read_only_vars, public_vars);

% --- 5. Zamek na startu - stabilizace po lokalizaci ---
if read_only_vars.counter <= 20
    public_vars.motion_vector = [0, 0]; % Prvnich 20 iteraci nejedeme
    
    % U EKF venku mu natvrdo dame uhel podle zacatku trasy, jinak by delal hodiny
    if public_vars.localization_converged && strcmp(public_vars.loc_mode, 'EKF') && ~isempty(public_vars.path)
        look_idx = min(5, size(public_vars.path, 1));
        target_dir = atan2(public_vars.path(look_idx, 2) - public_vars.mu(2), ...
                           public_vars.path(look_idx, 1) - public_vars.mu(1));
        
        public_vars.mu(3) = target_dir;
        public_vars.estimated_pose(3) = target_dir;
    end
end

end