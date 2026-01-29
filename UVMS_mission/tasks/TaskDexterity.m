classdef TaskDexterity < Task   
    properties
        id = "Dexterity Objective";
        mu_min;
        current_mu;
        q_center = [];
        err;
    end
    
    methods
        function obj = TaskDexterity(mu_min, robot)
            obj.mu_min = mu_min;
            obj.q_center = (robot.jlmax + robot.jlmin) / 2;
        end
        
        function updateReference(obj, robot)
            J_a = RobustJacobian(robot.q);
            obj.current_mu = sqrt(det(J_a * J_a'));
            obj.err = obj.mu_min - obj.current_mu;
            
            q_error = obj.q_center - robot.q;

            obj.xdotbar = 1.0 * q_error;
            obj.xdotbar = Saturate(obj.xdotbar(:), 0.2);
        end
        
        function updateJacobian(obj, robot)
            obj.J = [eye(7) zeros(7,6)];
        end
        
        function updateActivation(obj, robot)
            delta = 0.01;
            th = 0;
            obj.A = eye(7) * IncreasingBellShapedFunction(th,th + delta, 0, 1, obj.err);
        end
    end
end