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
            
            vel_threshold = 1e-2;
            speedXY = norm([vx, vy]);
            yaw_wv = atan2(vy, vx);
            
            if speedXY > vel_threshold
                R_target = [cos(yaw_wv) -sin(yaw_wv) 0; % yaw = yaw_velocity
                            sin(yaw_wv) cos(yaw_wv) 0;
                            0 0 1];
                wT_target = eye(4);
                wT_target(1:3,1:3) = R_target;
                wT_target(1:3,4) = robot.wTv(1:3,4);

                [ang, ~] = CartError(wT_target, robot.wTv);
                obj.xdotbar = 1 * ang;
                obj.xdotbar = Saturate(obj.xdotbar, 0.8);
            end
        end
        function updateJacobian(obj, robot)
            obj.J = [zeros(3,7) zeros(3,3) robot.wTv(1:3,1:3)];
        end
        function updateActivation(obj, robot)
            [~, lin] = CartError(robot.wTgv , robot.wTv);
            distToTarget = norm(lin(1:2));
            obj.A = eye(3) * IncreasingBellShapedFunction(1.0, 4.0, 0, 1, distToTarget);
        end
    end
end