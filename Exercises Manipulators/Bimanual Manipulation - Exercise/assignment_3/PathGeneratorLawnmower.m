function [waypoints_3d] = PathGeneratorLawnmower(robot_system, arm1, arm2)

    p_start = robot_system.left_arm.wTo(1:3, 4);

    L = 0.2; % Lunghezza passata (lungo X)
    W = 0.2; % Larghezza totale (lungo Y)
    passi = 4; % Numero di inversioni a U
    delta_y = W / passi;
    
    waypoints_2d = [];
    y_curr = 0;
    for i = 1:passi+1
        if mod(i, 2) ~= 0
            waypoints_2d = [waypoints_2d, [0; y_curr; 0], [L; y_curr; 0]];
        else
            waypoints_2d = [waypoints_2d, [L; y_curr; 0], [0; y_curr; 0]];
        end
        y_curr = y_curr + delta_y;
    end
    
    theta = deg2rad(-40);
    R_x = [1, 0, 0; 
           0, cos(theta), -sin(theta); 
           0, sin(theta), cos(theta)];
    
    rotated_waypoints = R_x * waypoints_2d;

    offset = p_start - rotated_waypoints(:, 1);
    
    waypoints_3d = rotated_waypoints + repmat(offset, 1, size(waypoints_2d, 2));

    wTog=[rotation(0,0,0) waypoints_3d(:, end)];
    arm1.set_obj_goal(wTog)
    arm2.set_obj_goal(wTog)
end

