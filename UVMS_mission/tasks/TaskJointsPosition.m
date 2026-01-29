classdef TaskJointsPosition < Task
    properties
        id = "Joints Home Position";
        q_home;
        e;
    end
    methods
        function obj = TaskJointsPosition(q_home)
            obj.q_home = q_home;
            obj.q_home = obj.q_home(:);
        end
        function updateReference(obj, robot)
            obj.e = obj.q_home - robot.q;
            obj.xdotbar = -0.5 * obj.e;
            obj.xdotbar = Saturate(obj.xdotbar(:), 0.2);
        end
        function updateJacobian(obj, robot)
            obj.J = [eye(7) , zeros(7,6)]; 
        end
        function updateActivation(obj, robot)
            obj.A = eye(7);
        end
    end
end