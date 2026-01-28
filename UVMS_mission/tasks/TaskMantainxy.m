classdef TaskMantainxy < Task   
    properties
        id = "Mantain XY";
        XY;
        err
    end


    methods
        
        function updateReference(obj, robot)
            obj.XY = robot.stablePos(1:2,4);
            actualXY = robot.wTv(1:2,4);
            obj.err = actualXY - obj.XY;
            obj.xdotbar = 0.2 * obj.err;
            obj.xdotbar(1:2) = Saturate(obj.xdotbar(1:2), 0.2);
        end
        function updateJacobian(obj, robot)
            obj.J = [zeros(2,7) robot.wTv(1:2,1:3) zeros(2,3)];
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(2) * IncreasingBellShapedFunction(0, 0.1, 0, 1, obj.err);
        end
    end
end