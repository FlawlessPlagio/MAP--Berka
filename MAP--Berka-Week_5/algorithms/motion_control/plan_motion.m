function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION - rizeni robota po trase

% init faze: cekani na GNSS data
if read_only_vars.counter <= 50
    public_vars.motion_vector = [0, 0];
    return;
end

pose = public_vars.estimated_pose;
path = public_vars.path;

if isempty(pose) || any(isnan(pose)) || isempty(path)
    public_vars.motion_vector = [0, 0];
    return;
end

% I. Pick navigation target
% hledani nejblizsiho bodu a aplikace lookahead vzdalenosti
dist = sqrt((path(:,1) - pose(1)).^2 + (path(:,2) - pose(2)).^2);
[min_dist, closest_idx] = min(dist);

lookahead_idx = min(closest_idx + 5, size(path, 1));
target = path(lookahead_idx, :); % cileny bod [x, y]

% II. Compute motion vector
% uhlova chyba k targetu
th_err = atan2(target(2) - pose(2), target(1) - pose(1)) - pose(3);
th_err = atan2(sin(th_err), cos(th_err)); % normalizace na <-pi, pi>

% P-regulator
Kp = 1.2;

% adaptivni rychlost (ochrana proti narazu pri velke chybe)
if abs(th_err) > 0.5
    v = 0.15;
else
    v = 0.5;
end

% podminka zastaveni v cili
if closest_idx == size(path, 1) && min_dist < 0.2
    v = 0;
end

w = Kp * th_err;
L = read_only_vars.agent_drive.interwheel_dist;

% inverzni kinematika
v_R = v + (w * L / 2);
v_L = v - (w * L / 2);

% saturace rychlosti dle limitu podvozku
max_v = read_only_vars.agent_drive.max_vel;
public_vars.motion_vector = [max(min(v_R, max_v), -max_v), max(min(v_L, max_v), -max_v)];

end