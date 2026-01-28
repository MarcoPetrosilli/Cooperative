classdef TaskLookAhead < Task   
    properties
        id = "Look Ahead";
    end


    methods
        function updateReference(obj, robot)

            if isempty(robot.v_nu) || length(robot.v_nu) < 3
                obj.xdotbar = zeros(3,1);
                return;
            end
            
            wRv = robot.wTv(1:3,1:3);
            w_vel_v = wRv * robot.v_nu(1:3);
            vx = w_vel_v(1);
            vy = w_vel_v(2);
            
            vel_threshold = 0.01;
            speedXY = norm([vx, vy]);
            yaw_wv = atan2(vy, vx);
            
            if speedXY > vel_threshold
                % vel_dir = w_vel_v / speedXY;
                % robot_x_axis = wRv(:, 1);
                % cross_prod = cross(robot_x_axis, vel_dir);
                % obj.xdotbar = 1 * cross_prod;
                % obj.xdotbar = Saturate(obj.xdotbar(1:3), 0.3);

                R_target = [cos(yaw_wv) -sin(yaw_wv) 0; % Roll = Pitch = 0, Yaw = yaw_velocity
                            sin(yaw_wv) cos(yaw_wv) 0;
                            0 0 1];
                wT_target = eye(4);
                wT_target(1:3,1:3) = R_target;
                wT_target(1:3,4) = robot.wTv(1:3,4);

                [ang, ~] = CartError(wT_target, robot.wTv);
                obj.xdotbar = 1 * ang;
                obj.xdotbar = Saturate(obj.xdotbar, 0.8);
            end



            % Desired Yaw
            % if speedXY > vel_threshold % if robot is moving linear
            %     yaw_wv_bar = atan2(vy, vx); %desired
            %     yaw_wv = atan2(wRv(2,1), wRv(1,1)); % current 
                % angle_error = atan2(sin(yaw_wv - yaw_wv_bar), cos(yaw_wv - yaw_wv_bar));
                % obj.xdotbar = [0; 0; -2.0 * angle_error]
                % obj.xdotbar = Saturate(obj.xdotbar, 0.3);
            % else
            %     obj.xdotbar = zeros(3,1); % keep current value
                


                % R_target = [cos(yaw_wv) -sin(yaw_wv) 0; % Roll = Pitch = 0, Yaw = yaw_velocity
                %             sin(yaw_wv) cos(yaw_wv) 0;
                %             0 0 1];
                % wT_target = eye(4);
                % wT_target(1:3,1:3) = R_target;
                % wT_target(1:3,4) = robot.wTv(1:3,4);
                % 
                % [ang, ~] = CartError(wT_target, robot.wTv);
                % obj.xdotbar = 0.8 * ang;
                % obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
            



            
        
        function updateJacobian(obj, robot)
            obj.J = [zeros(3,7) zeros(3,3) robot.wTv(1:3,1:3)];
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(3);
        end
    end
end