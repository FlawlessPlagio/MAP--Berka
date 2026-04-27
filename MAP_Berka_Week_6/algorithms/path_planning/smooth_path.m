function smoothed_path = smooth_path(path)
%SMOOTH_PATH - Iterativni vyhlazeni nalezene trasy (Task 3)

if isempty(path) || size(path, 1) < 3
    smoothed_path = path;
    return;
end

% Parametry vyhlazovace
weight_data = 0.5;   % Sila drzeni se puvodnich bodu
weight_smooth = 0.1; % Sila narovnavani trasy
tolerance = 0.00001;

smoothed_path = path;
change = tolerance;

while change >= tolerance
    change = 0.0;
    for i = 2:(size(path, 1) - 1)
        for j = 1:2
            aux = smoothed_path(i, j);
            smoothed_path(i, j) = smoothed_path(i, j) + ...
                weight_data * (path(i, j) - smoothed_path(i, j)) + ...
                weight_smooth * (smoothed_path(i-1, j) + smoothed_path(i+1, j) - 2.0 * smoothed_path(i, j));
            
            change = change + abs(aux - smoothed_path(i, j));
        end
    end
end

end