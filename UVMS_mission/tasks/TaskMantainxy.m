classdef TaskMantainxy < Task   
    properties
        id = "Mantain XY";
    end


    methods
        function updateReference(obj, robot)
            [~, lin] = CartError(robot.wTgv , robot.wTv);
            obj.xdotbar = 0.2 * lin(1:2);
            obj.xdotbar(1:2) = Saturate(obj.xdotbar(1:2), 0.2);
        end
        function updateJacobian(obj, robot)
            obj.J = [zeros(2,7) robot.wTv(1:2,1:3) zeros(2,3)];
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(2);
        end
    end
end