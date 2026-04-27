function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE - Hlavni ridici smycka simulace

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
    % Zamerne vypusteno init_particle_filter
    public_vars.gnss_history = [];
    public_vars.is_initialized = false;
    public_vars.kf.Q = zeros(2,2); 
    public_vars.mu = [2, 2, pi/2]; 
    public_vars.sigma = zeros(3,3);
end

% --- Sber dat a inicializace EKF (Kroky 1-51) ---
if read_only_vars.counter <= 50
    % Pridana ochrana: Sbirame jen platna data (ignorujeme NaN)
    if ~isempty(read_only_vars.gnss_position) && ~any(isnan(read_only_vars.gnss_position))
        public_vars.gnss_history = [public_vars.gnss_history; read_only_vars.gnss_position];
    end
    public_vars.estimated_pose = nan(1,3); % potlaceni vykreslovani ducha
    
elseif read_only_vars.counter == 51 && ~public_vars.is_initialized
    
    % Ochrana: Kdyby se stalo, ze 50 kroku jen stojime v cervene zone
    if isempty(public_vars.gnss_history)
        gnss_mean = [2, 2]; % Fallback pozice
        gnss_cov = [0.1, 0; 0, 0.1]; % Fallback kovariance
    else
        % Normalni vypocet z nasbiranych platnych dat
        gnss_cov = cov(public_vars.gnss_history);
        gnss_mean = mean(public_vars.gnss_history);
    end
    
    public_vars.kf.Q = gnss_cov;
    
    %  start
    public_vars.mu = [gnss_mean(1), gnss_mean(2), 0];
    public_vars.sigma = [gnss_cov(1,1), gnss_cov(1,2), 0;
                         gnss_cov(2,1), gnss_cov(2,2), 0;
                         0,             0,             100];
                         
    public_vars.is_initialized = true;
end
% ------------------------------------------------

% 9. Update particle filter
% Vypnuto

% 10. Update Kalman filter
if isfield(public_vars, 'is_initialized') && public_vars.is_initialized
    [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
end

% 11. Estimate current robot position
if isfield(public_vars, 'is_initialized') && public_vars.is_initialized
    public_vars.estimated_pose = public_vars.mu;
end

% 12. Path planning
public_vars.path = plan_path(read_only_vars, public_vars);

% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);

end