function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
    % priprava pole plneho NaN (kdyz paprsek netrefi zed)
    measurement = nan(1, length(lidar_config));
    
    start_x = pose(1);
    start_y = pose(2);
    robot_theta = pose(3);
    
    for i = 1:length(lidar_config)
        % absolutni uhel paprsku v mape
        ray_angle = robot_theta + lidar_config(i);
        
        % vrati matici pruseciku s prekazkama
        intersections = ray_cast([start_x, start_y], map.walls, ray_angle);
        
        % kdyz to neco trefi, najdeme to nejblizsi
        if ~isempty(intersections)
            vzdalenosti = sqrt((intersections(:,1) - start_x).^2 + (intersections(:,2) - start_y).^2);
            measurement(i) = min(vzdalenosti);
        end
    end
end