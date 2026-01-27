classdef TaskManipulabilityCheck < Task   
    properties
        id = "Manipulability check";
        arm_length;
        armBase_distance;
        correction_step;
        err;

    end

    methods
        function obj = TaskManipulabilityCheck(arm_reaach, armBase_vehicle_dist, correction_step)
            obj.arm_length = arm_reaach;
            obj.armBase_distance = armBase_vehicle_dist;
            obj.correction_step = correction_step;
        end
        function updateReference(obj, robot)
            w_Tool_goal_XY = robot.goalPosition(1:2); % nodule
            w_Vehicle_goal_XY = robot.vehicleGoalPosition(1:2);
            dist_goal_nodule = norm(w_Vehicle_goal_XY - w_Tool_goal_XY);
            lin = (w_Tool_goal_XY - w_Vehicle_goal_XY);

            obj.err = dist_goal_nodule - (obj.arm_length + obj.armBase_distance);
            if obj.err > 0 % If original goal is too far:
                fprintf("Manip adjust")
                
                
                obj.xdotbar = 0.8 * lin';
                obj.xdotbar(1:2) = Saturate(obj.xdotbar(1:2), 0.3);
            else
                obj.xdotbar(1:2) = [0 0]; % init values
            end
            % DEBUG
            fprintf('Size line: %d x %d\n', size(lin'));
            fprintf('Size xdotbar: %d x %d\n', size(obj.xdotbar));
            fprintf('Value xdotbar: [%.3f, %.3f]\n', obj.xdotbar(1), obj.xdotbar(2));
        end
        function updateJacobian(obj, robot)
            obj.J = [zeros(2,7) robot.wTv(1:2,1:3) zeros(2,3)];
        end
        
        function updateActivation(obj, robot)
            th = 0;
            delta = 0.5;
            act_value = IncreasingBellShapedFunction(th,th+delta,0,1,obj.err);
            obj.A = eye(2) * act_value;
            % DEBUG
            fprintf('Size A: %d x %d\n', size(obj.A));
        end
    end
end
