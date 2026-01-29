classdef TaskToolControl < Task   
    properties
        id = "Tool Control";
    end

    methods
        function updateReference(obj, robot)
            [ang, lin] = CartError(robot.vTg , robot.vTt);
            obj.xdotbar = 0.7 * [ang; lin];
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.25);
            obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.25);
        end
        function updateJacobian(obj, robot)
            bJe = RobustJacobian(robot.q);
            Ste = [eye(3) zeros(3);  -skew(robot.vTe(1:3,1:3)*robot.eTt(1:3,4)) eye(3)];
            Jt_a  = Ste * [robot.vTb(1:3,1:3) zeros(3,3); zeros(3,3) robot.vTb(1:3,1:3)] * bJe;
            Jt_v = zeros(6);
            obj.J = [Jt_a Jt_v];
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(6);
        end
    end
end