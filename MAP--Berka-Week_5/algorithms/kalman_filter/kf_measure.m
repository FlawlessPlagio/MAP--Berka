function [new_mu, new_sigma] = kf_measure(mu, sigma, z, kf)
%KF_MEASURE Summary of this function goes here

% matice mereni (pozorujeme x, y)
H = [1, 0, 0;
     0, 1, 0];

% Kalmanuv zisk
K = sigma * H' / (H * sigma * H' + kf.Q); 

% korekce stavu (inovace)
new_mu_col = mu(:) + K * (z(:) - H * mu(:));
new_mu = new_mu_col';

% korekce kovariance
new_sigma = (eye(3) - K * H) * sigma;

end