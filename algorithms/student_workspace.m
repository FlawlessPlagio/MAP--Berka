function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
          
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);

    %  INICIALIZACE TASK 2
        public_vars.lidar_history = [];
        public_vars.gnss_history = [];

end
% SBĚR DAT PRO TASK 2
    % data z LiDARu
    public_vars.lidar_history = [public_vars.lidar_history; read_only_vars.lidar_distances];
    
    % Ukládáme data z GNSS
    if ~isempty(read_only_vars.gnss_position)
        public_vars.gnss_history = [public_vars.gnss_history; read_only_vars.gnss_position];
    end

    %VÝPOČET A VYKRESLENÍ PO 120 ITERACÍCH
    if read_only_vars.counter == 120
        % Výpočet směrodatné odchylky
        sigma_lidar = std(public_vars.lidar_history);
        sigma_gnss = std(public_vars.gnss_history);
        
        disp('VÝSLEDKY TASK 2: ');
        disp('Standardní odchylka LiDARu (8 kanálů):');
        disp(sigma_lidar);
        disp('Standardní odchylka GNSS (X, Y):');
        disp(sigma_gnss);
        
        % Vykreslení histogramů 
        figure('Name', 'Task 2 - Sensor Noise Histograms');
        
        subplot(1,2,1);
        histogram(public_vars.lidar_history(:, 1), 15); %LiDAR
        title('Histogram - LiDAR (kanál 1)');
        xlabel('Vzdálenost (m)');
        ylabel('Četnost');
        
        subplot(1,2,2);
        histogram(public_vars.gnss_history(:, 1), 15); %GNSS
        title('Histogram - GNSS (osa X)');
        xlabel('Pozice X (m)');
        ylabel('Četnost');
    
    % Sestavení kovariančních matic
cov_lidar = cov(public_vars.lidar_history);
cov_gnss = cov(public_vars.gnss_history);

disp('--- VÝSLEDKY TASK 3 ---');
disp('Kovarianční matice LiDARu (8x8):');
disp(cov_lidar);

disp('Kovarianční matice GNSS (2x2):');
disp(cov_gnss);

% Ověření, že na diagonále je sigma
var_lidar_diag = diag(cov_lidar)';
var_gnss_diag = diag(cov_gnss)';

sigma2_lidar_manual = sigma_lidar.^2; 
sigma2_gnss_manual = sigma_gnss.^2;

disp('Ověření LiDAR (Diagonála z cov vs. sigma^2):');
disp([var_lidar_diag; sigma2_lidar_manual]);

disp('Ověření GNSS (Diagonála z cov vs. sigma^2):');
disp([var_gnss_diag; sigma2_gnss_manual]);
% --- VÝSLEDKY TASK 4 ---

        sigma_lidar_1 = sigma_lidar(1); % 1. kanál LiDARu 
        sigma_gnss_x = sigma_gnss(1);   % Osa X u GNSS
        
        % Nastavíme osu x od -2 do 2 metrů 
        x_vals = linspace(-2, 2, 1000);
        
        % Spočítáme PDF pro mu = 0 
        pdf_lidar = norm_pdf(x_vals, 0, sigma_lidar_1);
        pdf_gnss  = norm_pdf(x_vals, 0, sigma_gnss_x);
        
        % Vykreslíme obě křivky do jednoho obrázku
        figure('Name', 'Task 4 - Sensor Noise PDF');
        hold on; grid on;
        plot(x_vals, pdf_lidar, 'r', 'LineWidth', 2);
        plot(x_vals, pdf_gnss, 'b', 'LineWidth', 2);
        
        title('Probability Density Function (PDF) of Sensor Noise');
        xlabel('Chyba měření (m)');
        ylabel('Hustota pravděpodobnosti');
        legend('LiDAR (kanál 1)', 'GNSS (osa X)');
        hold off;
    end
% 9. Update particle filter
public_vars.particles = update_particle_filter(read_only_vars, public_vars);

% 10. Update Kalman filter
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% 11. Estimate current robot position
public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)

% 12. Path planning
public_vars.path = plan_path(read_only_vars, public_vars);

% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);



end

