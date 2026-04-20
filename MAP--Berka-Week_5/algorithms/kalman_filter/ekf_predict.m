function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, sampling_period)
%EKF_PREDICT - predikce stavu a kovariance pro dif. podvozek

% parametry a rychlosti
dt = sampling_period;
v = (u(1) + u(2)) / 2;
w = (u(1) - u(2)) / kf.L;
th = mu(3);

% nelinearni update stavu g(x,u)
new_mu = [mu(1) + v*cos(th)*dt, ...
          mu(2) + v*sin(th)*dt, ...
          atan2(sin(th + w*dt), cos(th + w*dt))];

% Jakobian G (parcialni derivace g podle stavu x)
G = [1, 0, -v*sin(th)*dt;
     0, 1,  v*cos(th)*dt;
     0, 0,  1];

% procesni sum R 
R = diag([0.00055, 0.00055, 0.00055]);

% update nejistoty
new_sigma = G * sigma * G' + R;

end