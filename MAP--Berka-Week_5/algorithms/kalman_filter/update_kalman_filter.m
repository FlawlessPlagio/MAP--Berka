function [mu, sigma] = update_kalman_filter(read_only_vars, public_vars)
%UPDATE_KALMAN_FILTER - EKF smycka (predikce a korekce)

mu = public_vars.mu;
sigma = public_vars.sigma;

% parametr podvozku pro predikci
public_vars.kf.L = read_only_vars.agent_drive.interwheel_dist;

% 1. Prediction 
u = public_vars.motion_vector;
if isempty(u)
    u = [0, 0]; % fallback pro stojiciho robota
end
[mu, sigma] = ekf_predict(mu, sigma, u, public_vars.kf, read_only_vars.sampling_period);

% 2. Measurement (GNSS - bezi jen kdyz je signal) ---
z = read_only_vars.gnss_position;

% Ochrana: Zkusit kf_measure jen, kdyz data nejsou prazdna a neobsahuji NaN
if ~isempty(z) && ~any(isnan(z))
    [mu, sigma] = kf_measure(mu, sigma, z, public_vars.kf);
end

end