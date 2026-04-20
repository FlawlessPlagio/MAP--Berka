function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)
    % 1. Načtení staré pozice částice
    x = old_pose(1);
    y = old_pose(2);
    theta = old_pose(3);

    % 2. Načtení parametrů
    dt = read_only_vars.sampling_period;
    L = read_only_vars.agent_drive.interwheel_dist;

    % 3. Načtení rychlostí z motion_vector 
    v_L = motion_vector(1) + randn() * 1.3; 
    v_R = motion_vector(2) + randn() * 1.3;

    % TASK 1: Probabilistic motion model 
    noise_std = 0.05; 
    v_R_noisy = v_R + randn() * noise_std;
    v_L_noisy = v_L + randn() * noise_std;

    % Výpočet lineární (v) a úhlové (omega) rychlosti
    v = (v_R_noisy + v_L_noisy) / 2;
    omega = (v_R_noisy - v_L_noisy) / L;

    % 4. Predikce nové polohy
    new_x = x + v * cos(theta) * dt;
    new_y = y + v * sin(theta) * dt;
    new_theta = theta + omega * dt;

    % Normalizace úhlu 
    new_theta = atan2(sin(new_theta), cos(new_theta));

    % 5. Výstup
    new_pose = [new_x, new_y, new_theta];
end