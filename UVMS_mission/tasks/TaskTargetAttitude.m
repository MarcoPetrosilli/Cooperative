classdef TaskTargetAttitude < Task   
    properties
        id = "Nodule Alignment";
    end


    methods
        function updateReference(obj, robot)
            wPv = robot.wTv(1:3,4);
            wPg = robot.wTg(1:3,4);
            dx = wPg(1) - wPv(1);
            dy = wPg(2) - wPv(2);
            yaw_target = atan2(dy, dx);

            % create a dynamic rotation matrix
            R_target = [cos(yaw_target) -sin(yaw_target) 0; % Roll = Pitch = 0, Yaw = yaw_target
                        sin(yaw_target) cos(yaw_target) 0;
                        0 0 1];
            wT_target = eye(4);
            wT_target(1:3, 1:3) = R_target;
            wT_target(1:3, 4) = wPv;
            
            [ang_err, ~] = CartError(wT_target, robot.wTv);
            obj.xdotbar = 0.4 * ang_err;
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end

        function updateJacobian(obj, robot)
            obj.J = [zeros(3,7) zeros(3,3) robot.wTv(1:3,1:3)];
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(3);
        end
    end
end