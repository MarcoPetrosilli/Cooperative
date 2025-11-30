classdef TaskDistance < Task   
    properties
        id = "distance";
    end


    methods
        function updateReference(obj, robot)
            [ang, lin] = CartError(robot.wTgv , robot.wTv);
            obj.xdotbar = -0.2 * lin;
            % limit thew requested velocities...
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.2);
        end
        function updateJacobian(obj, robot)
            obj.J = [zeros(3,7) -robot.wTv(1:3,1:3) zeros(3,3)];
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(3);
        end
    end
end