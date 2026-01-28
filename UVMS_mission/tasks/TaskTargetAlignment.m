classdef TaskTargetAlignment < Task   
    properties
        id = "Target Alignment";
        err;
    end

    methods
        function updateReference(obj, robot)
            wPv = robot.wTv(1:3,4);
            wPg = robot.wTg(1:3,4);
            dx = wPg(1) - wPv(1);
            dy = wPg(2) - wPv(2);
            yaw_target = atan2(dy, dx);

            % create a dynamic rotation matrix : yaw = yaw_target
            R_target = [cos(yaw_target) -sin(yaw_target) 0;
                        sin(yaw_target) cos(yaw_target) 0;
                        0 0 1];
            wT_target = eye(4);
            wT_target(1:3, 1:3) = R_target;
            wT_target(1:3, 4) = wPv;
            
            [ang_err, ~] = CartError(wT_target, robot.wTv);
            obj.err = norm(ang_err);
            obj.xdotbar = 0.8 * ang_err;
            obj.xdotbar = Saturate(obj.xdotbar, 0.3);
        end
        function updateJacobian(obj, robot)
            obj.J = [zeros(3,7) zeros(3,3) robot.wTv(1:3,1:3)];
        end
        function updateActivation(obj, robot)
            th = 0;
            delta = 0.3;
            obj.A = eye(3) * IncreasingBellShapedFunction(th, th + delta, 0, 1, obj.err);
        end
    end
end