classdef TaskTargetAlignment < Task   
    properties
        id = "Target Alignment";
        err;
    end

    methods
        function updateReference(obj, robot)
            [~ , lin_err] = CartError(robot.wTg, robot.wTv);
            yaw_target = atan2(lin_err(2),lin_err(1));

            % create a dynamic rotation matrix : yaw = yaw_target
            R_target = [cos(yaw_target) -sin(yaw_target) 0;
                        sin(yaw_target) cos(yaw_target) 0;
                        0 0 1];
            wT_target = eye(4);
            wT_target(1:3, 1:3) = R_target;
            wT_target(1:3, 4) = robot.wTv(1:3,4);
            
            [ang_err, ~] = CartError(wT_target, robot.wTv);
            obj.err = norm(ang_err);
            obj.xdotbar = 0.8 * ang_err;
            obj.xdotbar = Saturate(obj.xdotbar, 0.3);
        end
        function updateJacobian(obj, robot)
            obj.J = [zeros(3,7) zeros(3,3) robot.wTv(1:3,1:3)];
        end
        function updateActivation(obj, robot)
            obj.A = eye(3);
        end
    end
end