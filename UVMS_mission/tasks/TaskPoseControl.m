classdef TaskPoseControl < Task   
    properties
        id = "Pose Control";
        err;
    end

    methods
        function updateReference(obj, robot)
            [ang, lin] = CartError(robot.wTgv , robot.wTv);
            obj.xdotbar = 0.4 * [ang; lin];
            obj.err = norm(lin(1:2));
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.3);
            obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.3);
        end
        function updateJacobian(obj, robot)
            obj.J = [zeros(3,7) zeros(3,3) robot.wTv(1:3,1:3);
                zeros(3,7) robot.wTv(1:3,1:3) zeros(3,3)];
        end
        function updateActivation(obj, robot)
            obj.A = eye(6);
        end
    end
end
